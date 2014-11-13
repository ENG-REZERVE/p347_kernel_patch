/*
    p347 test fpga driver
    
    uses GPMC, SPI, GPIO
    uses _PDB instead of printk for detailed information
*/

#include <linux/p347_fpga.h>

#define GPMC_CS0		0x60
#define GPMC_CS_SIZE		0x30

#define p347_FPGA_IRQ_ENABLE	__raw_writel(1<<p347_GPIO_BITPOS,gpio_base + 0x4064)
#define p347_FPGA_IRQ_DISABLE	__raw_writel(1<<p347_GPIO_BITPOS,gpio_base + 0x4060)

static void* d_ret = NULL;

static p347_fpga_device_t	p347_fpga_info;

//static struct gpmc_timings g_time;
static unsigned long fpga_mem_base;
static unsigned long fpga_mem_remap;

static void __iomem *gpmc_base;// = 0x6e000000;
static struct clk *gpmc_l3_clk = NULL;
static struct clk *dpll3_clk = NULL;

//==========================================================================
//==========================================================================
//==========================================================================

u32 gpmc_cs_read_reg(int cs, int idx)
{
	void __iomem *reg_addr;

	reg_addr = gpmc_base + GPMC_CS0 + (cs * GPMC_CS_SIZE) + idx;
	return __raw_readl(reg_addr);
}

void my_gpmc_cs_write_reg(int cs, int idx, u32 val)
{
    void __iomem *reg_addr;

    reg_addr = gpmc_base + GPMC_CS0 + (cs * GPMC_CS_SIZE) + idx;
    __raw_writel(val, reg_addr);
}

//==========================================================================
//==========================================================================
//==========================================================================

static irqreturn_t p347_fpga_irq_handler(int irq, void* dev_id)
{
    //int i;
    //unsigned short d;
    p347_fpga_info.i_cnt++;

    _PDBA("p347_fpga_irq_handler irq=%d\n",irq);
    
    //DMA
    memset(d_ret,0,RD_BURST_LEN*2);
    omap_start_dma(p347_fpga_info.dma_ch);

    //noDMA
/*    memset(&p347_fpga_info.DST[0],0,RD_BURST_LEN*2);
    d = __raw_readw(fpga_mem_remap);
    if (d != 0x51AC) p347_fpga_info.err_cnt++;
    _PDBA("rec = 0x%4x, IRQ #%d, errcnt =%d\n",d,p347_fpga_info.i_cnt,p347_fpga_info.err_cnt);

    for (i=0; i<RD_BURST_LEN; i++) 
	p347_fpga_info.DST[i] =__raw_readw(fpga_mem_remap);	
    for (i=0; i<RD_BURST_LEN; i++)
	_PDBA("rec[%d] = 0x%4x\n",i,p347_fpga_info.DST[i]);
*/
    return IRQ_HANDLED;
}

static void p347_fpga_dma_callback(int lch, u16 ch_status, void* data)
{
    int i;
    _PDBA("p347_fpga_dma_callback lch=%d, ch_status=0x%x, data=0x%p\n",lch,ch_status,(void*)data);
    //TODO: dma finished burst reading, need to reply
    omap_stop_dma(p347_fpga_info.dma_ch);
    
    _PDB("--------------------------------\n");
    for (i=0;i<RD_BURST_LEN;i++) _PDBA("rec[%d] = 0x%4x\n",i,((unsigned short*)(d_ret))[i]);
    _PDB("--------------------------------\n");
}

//==========================================================================
//==========================================================================
//==========================================================================

static int p347_fpga_open(struct inode *inode, struct file *filp)
{
    _PDBA("p347_fpga_open: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);

    if (p347_fpga_info.opened > 0) {
	_PDB("ERROR: device is already opened!\n");
	return -EBUSY;
    }
    p347_fpga_info.opened=1;
    filp->private_data = &p347_fpga_info;

    //TODO: enable it here
    //omap_enable_irq_dma(p347_fpga_info.dma_ch);

    return nonseekable_open(inode,filp);
}

//==========================================================================

static int p347_fpga_release(struct inode *inode, struct file *filp)
{
    _PDBA("p347_fpga_release: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);
    p347_fpga_info.opened=0;

    return 0;
}

//==========================================================================

static int p347_fpga_ioctl(struct inode *inode, struct file *filp, uint cmd, unsigned long arg)
{
    t_gpmc_data*	setup_g;
    t_spi_data*		setup_s;
    char kdata[20];
    int wcnt,ret = 0;
    unsigned short* us = (unsigned short*)&kdata[0];
    
    memset(&kdata[0],0,20);
    _PDBA("p347_fpga_ioctl start: inode=0x%p, filp=0x%p, cmd=%d, arg=0x%p\n",(void*)inode,(void*)filp,cmd,(void*)arg);
    switch (cmd) {
	//----------------------------------------------------------------------------------------------GPMC
	case p347_IOCTL_WRITE: { //test write
	    if (arg != 0) {
		setup_g = (t_gpmc_data*)arg;
		ret = copy_from_user(&kdata[0],setup_g->data,setup_g->word_cnt*sizeof(u16));
		if (ret > 0) _PDBA("p347_IOCTL_READ: cannot copy %d bytes from userspace\n",ret);
		for (wcnt=0; wcnt<setup_g->word_cnt; wcnt++)
		    __raw_writew(&kdata[wcnt*2],fpga_mem_remap);
	    }
	break; }
	case p347_IOCTL_READ: {
	    if (arg != 0) {
		setup_g = (t_gpmc_data*)arg;
		*us = __raw_readw(fpga_mem_remap);
		ret = copy_to_user(setup_g->data,&kdata[0],2);
		if (ret > 0) _PDBA("p347_IOCTL_READ: cannot copy %d bytes to userspace\n",ret); 
		_PDBA("READ %02x\n",*us);
	    }
	break; }
	//----------------------------------------------------------------------------------------------SPI
	case p347_IOCTL_SPI_WRITE: { //test spi write, answer is not supposed
	    if (arg != 0) {
		setup_s = (t_spi_data*)arg;
		ret = copy_from_user(p347_fpga_info.spi_tx_buff,setup_s->data_tx,setup_s->word_cnt*sizeof(u32));
		if (ret > 0) _PDBA("p347_IOCTL_READ: cannot copy %d bytes from userspace\n",ret);
		p347_fpga_info.transfer.len = setup_s->word_cnt;
		
		spi_message_init(&p347_fpga_info.msg);
		spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);
		
		//sleep until complete
		//TODO: test, think about hanging up, do something
		ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
		_PDBA("spi_sync in SPI_WRITE ret=%d\n",ret);
	    }
	break; }
	case p347_IOCTL_SPI_SENDCMD: { //send command and wait for answer
	    if (arg != 0) {
		setup_s = (t_spi_data*)arg;
		ret = copy_from_user(p347_fpga_info.spi_tx_buff,setup_s->data_tx,setup_s->word_cnt*sizeof(u32));
		if (ret > 0) _PDBA("p347_IOCTL_READ: cannot copy %d bytes from userspace\n",ret);
		p347_fpga_info.transfer.len = setup_s->word_cnt;
		
		spi_message_init(&p347_fpga_info.msg);
		spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);
		
		//sleep until complete
		//TODO: test, think about hanging up, do something
		ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
		_PDBA("spi_sync in SPI_SENDCMD ret=%d\n",ret);
		ret = copy_to_user(setup_s->data_rx,p347_fpga_info.spi_rx_buff,setup_s->word_cnt*sizeof(u32));
		if (ret > 0) _PDBA("p347_IOCTL_READ: cannot copy %d bytes to userspace\n",ret);
		for (wcnt=0; wcnt<setup_s->word_cnt; wcnt++)
		    _PDBA("SPI answer[%d]=0x%x\n",wcnt,p347_fpga_info.spi_rx_buff[wcnt]);
	    }
	break; }
	case p347_IOCTL_SPI_SET_HZ: {
	    if ((arg >= p347_SPI_MIN_SPEED) && (arg <= p347_fpga_info.spidev->max_speed_hz)) {
		_PDBA("Modify spi transfer speed from %ld to %ld\n",p347_fpga_info.transfer.speed_hz,arg);
		p347_fpga_info.transfer.speed_hz = arg;
	    } else {
		_PDBA("Incorrect input HZ for SPI, should be between 1.5kHZ and %d\n",p347_fpga_info.spidev->max_speed_hz);
		_PDB("You must use speed definitions from p347_fpga_user.h\n");
	    }
	break; }
	//----------------------------------------------------------------------------------------------Client
	case p347_IOCTL_CLIENT_REGISTER: {
	
	break; }
	case p347_IOCTL_CLIENT_UNREGISTER: {
	
	break; }
	default: {
	    _PDB("unknown cmd\n");
	break; }
    };
    _PDB("p347_fpga_ioctl end\n");
    return 0;
}

//==========================================================================

static struct file_operations p347_fpga_fops = {
    owner:	THIS_MODULE,
    ioctl:	p347_fpga_ioctl,
    open:	p347_fpga_open,
    release:	p347_fpga_release,
    fasync:	NULL,
    llseek:	no_llseek,
};

//==========================================================================
//==========================================================================
//==========================================================================

static void p347_spi_complete(void)
{
    _PDB("p347_spi_complete call\n");
}

static int __devinit p347_spi_probe(struct spi_device *spi)
{
    int ret;
    _PDBA("__devinit p347_spi_probe, spi=0x%p\n",(void*)spi);
    p347_fpga_info.spidev = spi;
    
    spi->bits_per_word = 32;
    spi->mode = SPI_MODE_2;
    ret = spi_setup(spi);
    _PDBA("spi_setup ret %d\n",ret);
    
    spi_message_init(&p347_fpga_info.msg);
    p347_fpga_info.msg.complete = p347_spi_complete;
    
    return ret;
};

//==========================================================================

static int __devexit p347_spi_remove(struct spi_device *spi)
{
    _PDB("__devexit p347_spi_remove\n");
    
    return 0;
};

//==========================================================================

static struct spi_driver p347_spi_driver = {
    .driver = {
	.name 	= "p347_spi",
	.bus 	= &spi_bus_type,
	.owner 	= THIS_MODULE,
    },
    .probe 	= p347_spi_probe,
    .remove	= __devexit_p(p347_spi_remove),
    .suspend	= NULL,
    .resume	= NULL,
};

//==========================================================================
//==========================================================================
//==========================================================================

#define set_read_print(cs,reg,var,name) \
    my_gpmc_cs_write_reg(cs, reg, var);\
    var = gpmc_cs_read_reg(cs, reg);\
    _PDBA(name" reg value = 0x%x\n",var);

int __init p347_fpga_init(void)
{
    int ret;
    u32 reg_val;
    size_t tmp_sz = 0;
    unsigned char div,mult;
    unsigned long rate;
    unsigned long scm_base = ioremap(0x48000000,SZ_4K);
    unsigned long cm_addr = scm_base + 0x4d40;
    unsigned long padconf_addr;
    unsigned long gpio_base = ioremap(0x49050000,SZ_4K);    
    unsigned long dmareg_base = ioremap(0x48056000,SZ_4K);
    
    p347_fpga_info.err_cnt = 0;
    //ret = irq_to_gpio(161);
    //_PDBA("irq 161 is gpio%d\n",ret);
    
    //TODO: maybe SZ_16M is not needed
    ret = gpmc_cs_request(p347_GPMC_FPGA_CS,SZ_16M,&fpga_mem_base);
    _PDBA("gpmc_cs_request ret=%d\n",ret);
    fpga_mem_remap = ioremap(fpga_mem_base,SZ_4K);
    
    _PDB("p347_fpga_init start\n");
    _PDBA("mem base=0x%p, remap=0x%p\n",(void*)fpga_mem_base,(void*)fpga_mem_remap);
    memset(&p347_fpga_info,0,sizeof(p347_fpga_device_t));
    p347_fpga_info.i_cnt = 0;

    //-----------------------------------------------------------memory allocations

    tmp_sz = RD_BURST_LEN*2;
    d_ret = dma_alloc_coherent(NULL,tmp_sz,&p347_fpga_info.dma_rd_addr,GFP_DMA);
    if (d_ret == NULL) {
	_PDBA("Cannot allocate space for gpmc dma rx buffer (%d bytes)\n",tmp_sz);
	return -ENOMEM;
    }
    _PDBA("dma_alloc_coherent ret=0x%08x, addr =0x%08x\n",d_ret,p347_fpga_info.dma_rd_addr);
    
    tmp_sz = SPI_RXBUFF_LEN*sizeof(u32);
    p347_fpga_info.spi_rx_buff = kzalloc(tmp_sz,GFP_KERNEL);
    if (p347_fpga_info.spi_rx_buff == NULL) {
	_PDBA("Cannot allocate space for spi_rx_buff (%d bytes)\n",tmp_sz);
	dma_free_coherent(NULL,RD_BURST_LEN*2,d_ret,p347_fpga_info.dma_rd_addr);
	return -ENOMEM;
    }
    _PDBA("spi_rx_buff kzallocated in 0x%08x\n",p347_fpga_info.spi_rx_buff);
    
    tmp_sz = SPI_TXBUFF_LEN*sizeof(u32);
    p347_fpga_info.spi_tx_buff = kzalloc(tmp_sz,GFP_KERNEL);
    if (p347_fpga_info.spi_tx_buff == NULL) {
	_PDBA("Cannot allocate space for spi_tx_buff (%d bytes)\n",tmp_sz);
	dma_free_coherent(NULL,RD_BURST_LEN*2,d_ret,p347_fpga_info.dma_rd_addr);
	kfree(p347_fpga_info.spi_rx_buff);
	return -ENOMEM;
    }
    _PDBA("spi_tx_buff kzallocated in 0x%08x\n",p347_fpga_info.spi_tx_buff);
    
    //-----------------------------------------------------------gpmc_clk pad configuration
    padconf_addr = scm_base + PADCONF_GPMC_CLK_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read clk padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    reg_val &= 0x0000FFFF;
    reg_val |= 0x01100000;
    __raw_writel(reg_val,padconf_addr);
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read clk padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    
    //-----------------------------------------------------------fpga_irq pad configuration
    padconf_addr = scm_base + PADCONF_FPGA_IRQ_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    if (PADCONF_FPGA_IRQ_OFFSET) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;
    reg_val |= (PADCONF_FPGA_IRQ_VALUE << PADCONF_FPGA_IRQ_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);

    //one more cam padconf, not gpio (cam_strobe), not used for now
/*    padconf_addr = scm_base + 0x2130;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0x0000FFFF;
    reg_val |= 0x011C0000;
    __raw_writel(reg_val,padconf_addr);
*/    
    //-----------------------------------------------------------dpll3 clock divider (make low rate)
    reg_val = __raw_readl(cm_addr);
    _PDBA("read addr 0x%p ret 0x%x\n",(void*)cm_addr,reg_val);
    reg_val &= 0x07FFFFFF;
    reg_val |= (16 << 27);
    __raw_writel(reg_val,cm_addr);
    reg_val = __raw_readl(cm_addr);
    _PDBA("read addr 0x%p ret 0x%x\n",(void*)cm_addr,reg_val);

    gpmc_base = ioremap(0x6e000000,SZ_4K);
    _PDBA("ioremap ret=0x%p\n",(void*)gpmc_base);

    gpmc_l3_clk = clk_get(NULL,"gpmc_fck");
    if (IS_ERR(gpmc_l3_clk)) {
	_PDB("Could not get GPMC clock for p347_fpga\n");
    }
    
    ret = clk_enable(gpmc_l3_clk);
    _PDBA("gpmc_l3_clk enable ret %d\n",ret);
    
    rate = clk_get_rate(gpmc_l3_clk);
    if (rate == 0) {
	printk(KERN_WARNING "gpmc_l3_clk not enabled\n");
	return -1;
    } else {
	_PDBA("gpmc_fck rate=%d\n",rate);
    }

    dpll3_clk = clk_get(NULL,"dpll3_m2_ck");
    if (IS_ERR(dpll3_clk)) {
	_PDB("Could not get dpll3 clock for p347_fpga\n");
    }

    rate = clk_get_rate(dpll3_clk);
    if (rate == 0) {
	printk(KERN_WARNING "dpll3_m2_ck not enabled\n");
	//return -1;
    } else {
	_PDBA("dpll3_m2_ck rate=%d\n",rate);
    }

    //-----------------------------------------------------------request pins
    
    ret = gpio_request(p347_GPIO_FPGA_IRQ,"gpio_fpga_irq");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_fpga_irq, ret=%d\n",ret);
	return ret;
    }
    _PDB("gpio_fpga_irq registered\n");
    
    reg_val = __raw_readl(gpio_base + 0x404C); //GPIO4 FALLINGDETECT
    reg_val |= (1 << p347_GPIO_BITPOS);
    __raw_writel(reg_val,gpio_base + 0x404C);
    
    p347_fpga_info.irqnum = gpio_to_irq(p347_GPIO_FPGA_IRQ);
    _PDBA("gpi_to_req ret=%d\n",p347_fpga_info.irqnum);
    
    ret = request_irq(p347_fpga_info.irqnum,p347_fpga_irq_handler,0,"p347_fpga_irq",&p347_fpga_info);
    _PDBA("request_irq %d ret %d\n",p347_fpga_info.irqnum,ret);
    
    //-----------------------------------------------------------dma setup
    
    p347_fpga_info.dma_par.data_type = OMAP_DMA_DATA_TYPE_S16;
    //p347_fpga_info.dma_par.src_start = fpga_mem_remap;
    p347_fpga_info.dma_par.src_start = fpga_mem_base;
    p347_fpga_info.dma_par.src_amode = OMAP_DMA_AMODE_POST_INC;
    //p347_fpga_info.dma_par.dst_start = &p347_fpga_info.DST[0];
    p347_fpga_info.dma_par.dst_start = p347_fpga_info.dma_rd_addr;
    _PDBA("dst addr = 0x%08x\n",p347_fpga_info.dma_par.dst_start);
    p347_fpga_info.dma_par.dst_amode = OMAP_DMA_AMODE_POST_INC;
    p347_fpga_info.dma_par.dst_ei = 1;
    p347_fpga_info.dma_par.dst_fi = 1;
    //p347_fpga_info.dma_par.src_ei = 1;
    //p347_fpga_info.dma_par.src_fi = 1;
    p347_fpga_info.dma_par.elem_count = 8;
    //p347_fpga_info.dma_par.elem_count = 1;
    p347_fpga_info.dma_par.frame_count = 1;
    p347_fpga_info.dma_par.sync_mode = OMAP_DMA_SYNC_FRAME;
    //p347_fpga_info.dma_par.src_or_dst_sync = OMAP_DMA_SRC_SYNC;
/*    
    //dev_id = 0 ??? , data* = NULL ???
    p347_fpga_info.dma_ch = -1;
    ret = omap_request_dma(77,"p347_fpga_read_dma",p347_fpga_dma_callback,NULL,&p347_fpga_info.dma_ch);
    _PDBA("omap_request_dma ret %d, channel = %d\n",ret,p347_fpga_info.dma_ch);
    
    omap_set_dma_params(p347_fpga_info.dma_ch,&p347_fpga_info.dma_par);
    omap_set_dma_src_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_8);
    omap_set_dma_dest_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_8);
*/    
/*    
    ret = gpio_request(p347_GPIO_FPGA_CS,"gpio_fpga_cs");
    if (ret != 0) {
	gpio_free(p347_GPIO_FPGA_IRQ);
	_PDB("ERROR: cannot request gpio_fpga_cs, ret=%d\n",ret);
	return ret;
    }
    _PDB("gpio_fpga_cs registered\n");
*/
    //-----------------------------------------------------------gpmc request and setup
    
    //set config1
    //my_gpmc_cs_write_reg(p347_GPMC_FPGA_CS,GPMC_CS_CONFIG1,0x28001000);
    
    //set timings
    
    div = 4; //166->41.5 mhz
    mult = 2; //x2 latencies, and x2 more will be in TIMEPARAGRANULARITY
    
    //cswrofftime = 5 ticks, csrdofftime = 15 ticks, csextradelay = 0, csontime = 0
    reg_val = (5*mult << 16) | (15*mult << 8) | (0*mult << 7) | (0*mult);
    //my_gpmc_cs_write_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG2, reg_val);
    set_read_print(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG2, reg_val,"CONFIG2");
    //advwrofftime = 2, advrdofftime = 4, advextradelay = 0, advontime = 1
    reg_val = (2*mult << 16) | (4*mult << 8) | (0*mult << 7) | (1*mult);
    //my_gpmc_cs_write_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG3, reg_val);
    set_read_print(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG3, reg_val,"CONFIG3");
    //weofftime = 4, weextradelay = 0, weontime = 3, oeofftime = 7, oeextradelay = 0, oeontime = 4
    reg_val = (4*mult << 24) | (0*mult << 23) | (3*mult << 16) | (13*mult << 8) | (0*mult << 7) | (4*mult);
    //my_gpmc_cs_write_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG4, reg_val);
    set_read_print(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG4, reg_val,"CONFIG4");
    //pageburstaccesstime = 1, rdaccesstime = 4, wrcycletime = 7, rdcycletime = 15
    reg_val = (1*mult << 24) | (4*mult << 16) | (7*mult << 8) | (15*mult);
    //my_gpmc_cs_write_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG5, reg_val);
    set_read_print(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG5, reg_val,"CONFIG5");
    //wraccesstime = 7, wrdataonadmuxbus = 2, cycle2cycledelay = 1
    //cycle2cyclesamecsen = 0, cycle2cyclediffcsen = 1, busturnaround = 1
    reg_val = gpmc_cs_read_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG6);
    reg_val &= 0x80000000; //do not modify bit 31
    reg_val |= (7*mult << 24) | (2*mult << 16) | (1*mult << 8) | (0*mult << 7) | (1*mult << 6) | (1*mult);
    //my_gpmc_cs_write_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG6, reg_val);
    set_read_print(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG6, reg_val,"CONFIG6");
    //and main config
    reg_val = gpmc_cs_read_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG1);
    //1) fclk to clk divider, div = 1,2,3,4
    reg_val &= ~0x03; reg_val |= (div - 1);
    //2) access
    reg_val &= ~0x01800000; reg_val |= 0x00800000; //0x01 = 8 words in burst
    reg_val |= 0x08000000; //writetype = synchronous, single access
    reg_val |= 0x60000000; //readtype = synchronous, multiple access
    if (div == 4) reg_val |= 0x10; //TIMEPARAGRANULARITY
    //my_gpmc_cs_write_reg(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG1, reg_val);
    set_read_print(p347_GPMC_FPGA_CS, GPMC_CS_CONFIG1, reg_val,"CONFIG1");


    p347_fpga_info.dma_ch = -1;
    ret = omap_request_dma(77,"p347_fpga_read_dma",p347_fpga_dma_callback,NULL,&p347_fpga_info.dma_ch);
    _PDBA("omap_request_dma ret %d, channel = %d\n",ret,p347_fpga_info.dma_ch);
    
    omap_set_dma_params(p347_fpga_info.dma_ch,&p347_fpga_info.dma_par);
    omap_set_dma_src_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_8);
    omap_set_dma_dest_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_8);
    //omap_set_dma_src_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_4);
    //omap_set_dma_dest_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_4);

    _PDBA("GCR = 0x%08x\n",__raw_readl(dmareg_base+0x78));
    _PDBA("CCR2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x80));
    _PDBA("CLINK2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x84));
    _PDBA("CICR2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x88));
    _PDBA("CSR2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x8C));
    _PDBA("CSDP2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x90));
    _PDBA("CEN2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x94));
    _PDBA("CFN2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x98));
    _PDBA("CSSA2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0x9C));
    _PDBA("CDSA2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xA0));
    _PDBA("CSEL2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xA4));
    _PDBA("CSFL2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xA8));
    _PDBA("CDEL2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xAC));
    _PDBA("CDFL2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xB0));
    _PDBA("CSAC2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xB4));
    _PDBA("CDAC2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xB8));
    _PDBA("CCEN2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xBC));
    _PDBA("CCFN2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xC0));
    _PDBA("COLOR2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xC4));
    _PDBA("CDP2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xD0));
    _PDBA("CNDP2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xD4));
    _PDBA("CCDN2 = 0x%08x\n",__raw_readl(dmareg_base+2*0x60+0xD8));
    
    reg_val = __raw_readl(dmareg_base+2*0x60+0x90); //CSDP2
    reg_val |= 0x40; //bit6 - source provided packed data
    __raw_writel(reg_val,dmareg_base+2*0x60+0x90);
    
    
    ret = register_chrdev(p347_CHARDEV_NUM,p347_CHARDEV_NAME,&p347_fpga_fops);
    if (ret < 0) {
	//gpio_free(p347_GPIO_FPGA_CS);
	gpio_free(p347_GPIO_FPGA_IRQ);
	_PDB("ERROR: cannot register char device for p347_fpga\n");
	return ret;
    }

    //TODO: disable it here (enable in _open)
    omap_enable_irq_dma(p347_fpga_info.dma_ch);

    padconf_addr = scm_base + PADCONF_FPGA_IRQ_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);

    reg_val = __raw_readl(gpio_base + 0x4010); //GPIO_SYSCONFIG
    _PDBA("SYSCONFIG for GPIO4 = 0x%08x\n",reg_val);
    reg_val = __raw_readl(gpio_base + 0x4014); //GPIO_SYSSTATUS
    _PDBA("SYSSTATUS for GPIO4 = 0x%08x\n",reg_val);
    reg_val = __raw_readl(gpio_base + 0x4034); //GPIO_OE
    _PDBA("OE for GPIO4 = 0x%08x\n",reg_val);
    reg_val = __raw_readl(gpio_base + 0x4038); //GPIO_DATAIN
    _PDBA("DATAIN for GPIO4 = 0x%08x\n",reg_val);
    reg_val = __raw_readl(gpio_base + 0x404C); //GPIO_FALLINGDETECT
    _PDBA("FALLINGDETECT for GPIO4 = 0x%08x\n",reg_val);
    reg_val = __raw_readl(gpio_base + 0x401C); //GPIO_IRQENABLE1
    _PDBA("IRQENABLE1 for GPIO4 = 0x%08x\n",reg_val);

    //-----------------------------------------------------------
    //TODO: move to board description
    ret = spi_register_driver(&p347_spi_driver);
    _PDBA("spi_register_driver ret=%d\n",ret);
    //-----------------------------------------------------------

    _PDB("p347_fpga_init end\n");
    return 0;
}

//==========================================================================

void p347_fpga_exit(void)
{
    _PDB("p347_fpga_exit start\n");
    
    spi_unregister_driver(&p347_spi_driver);
    
    unregister_chrdev(p347_CHARDEV_NUM,p347_CHARDEV_NAME);
    
    omap_stop_dma(p347_fpga_info.dma_ch);
    omap_free_dma(p347_fpga_info.dma_ch);
    free_irq(p347_fpga_info.irqnum,&p347_fpga_info);

    //gpio_free(p347_GPIO_FPGA_CS);
    gpio_free(p347_GPIO_FPGA_IRQ);
    gpmc_cs_free(p347_GPMC_FPGA_CS);

    kfree(p347_fpga_info.spi_rx_buff);
    kfree(p347_fpga_info.spi_tx_buff);
    dma_free_coherent(NULL,RD_BURST_LEN*2,d_ret,p347_fpga_info.dma_rd_addr);

    //platform_device_unregister(p347_fpga_info->pdev);
    _PDB("p347_fpga_exit end\n");
}

//==========================================================================
//==========================================================================
//==========================================================================
module_init(p347_fpga_init);
module_exit(p347_fpga_exit);
//==========================================================================
MODULE_AUTHOR("Konstantin Utkin");
MODULE_LICENSE("GPL");
//==========================================================================
