/*
    p347 spi test fpga driver
    
    uses SPI, GPIO
    uses _PDB instead of printk for detailed information
*/

#include <linux/p347_spi_test.h>

static p347_fpga_device_t	p347_fpga_info;

static struct clk *gpmc_l3_clk = NULL;
static struct clk *dpll3_clk = NULL;

static spinlock_t irq_lock;

static t_p347_inner	tpi;

static irqreturn_t p347_fpga_irq_handler(int irq, void* data)
{
    unsigned long flags;

    spin_lock_irqsave(&irq_lock,flags);
    p347_fpga_info.i_cnt++; //count interrupts
    spin_unlock_irqrestore(&irq_lock,flags);

    return IRQ_HANDLED;
}

//==========================================================================
//==========================================================================		GPMC
//==========================================================================

//TODO: maybe user should know which register exactly causes error
#define TRY_SET(_idx_,_val_) \
    ret = spi_write_adc_register(adc_idx,_idx_,_val_);\
    if (ret != 0) return ret

#define TRY_SET_FPGA(_idx_,_val_)\
    ret = spi_write_fpga_register(_idx_,_val_);\
    if (ret != 0) return ret

//==========================================================================
//==========================================================================		COMMON
//==========================================================================

static int p347_fpga_open(struct inode *inode, struct file *filp)
{
    _PDBA("p347_fpga_open: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);

    if (p347_fpga_info.opened > 0) {
	printk("ERROR: device is already opened!\n");
	return -EBUSY;
    }
    p347_fpga_info.opened=1;
    //filp->private_data = &p347_fpga_info;

    //printk("FPGA_RESET=>1\n");
    //gpio_set_value(p347_GPIO_FPGA_RESET,1);

    return nonseekable_open(inode,filp);
}

//==========================================================================

static int p347_fpga_release(struct inode *inode, struct file *filp)
{
    _PDBA("p347_fpga_release: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);
    p347_fpga_info.opened=0;

    //printk("FPGA_RESET=>0\n");
    //gpio_set_value(p347_GPIO_FPGA_RESET,0);

    return 0;
}

//==========================================================================
//==========================================================================		SPI
//==========================================================================
//Basic SPI logic functions, may be called from ioctl or from inner logic process
// (NOT AT THE SAME TIME FROM BOTH!)

//adc_idx 1..4, reg_idx 1..N
int spi_read_adc_register(unsigned char adc_idx, unsigned char reg_idx, unsigned short* reg_val)
{
    int ret = 0;
    unsigned long	value = 0; //read

    p347_fpga_info.spi_tx_buff[0] = (adc_idx << 24) | (reg_idx << 16);
    p347_fpga_info.spi_tx_buff[1] = 0;
    p347_fpga_info.spi_tx_buff[2] = 0;
    //p347_fpga_info.spi_tx_buff[3] = 0;
    //p347_fpga_info.spi_tx_buff[4] = 0;
    memset(&p347_fpga_info.spi_rx_buff[0],0,SPI_RXBUFF_LEN*sizeof(u32));

    spi_message_init(&p347_fpga_info.msg);
    //p347_fpga_info.transfer.len = 5*sizeof(u32); //5 words (cmd,wait,wait,wait,get)
    p347_fpga_info.transfer.len = 3*sizeof(u32);
    p347_fpga_info.transfer.cs_change = 1;
    p347_fpga_info.transfer.bits_per_word = 32;
    p347_fpga_info.transfer.speed_hz = p347_fpga_info.new_hz;
    p347_fpga_info.transfer.tx_buf = p347_fpga_info.spi_tx_buff;
    p347_fpga_info.transfer.rx_buf = p347_fpga_info.spi_rx_buff;

    spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);

    //sleep until complete
    //TODO: test, think about hanging up, do something
    ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
    //_PDBA("spi_sync in spi_read_adc_register ret=%d\n",ret);
    if (ret != 0) return ret;

    //*reg_val = (p347_fpga_info.spi_rx_buff[4] & 0x0000FFFF);
    *reg_val = (p347_fpga_info.spi_rx_buff[2] & 0x0000FFFF);
    _PDBA("Reg value = 0x%04x\n",*reg_val);

    return 0;
}

int spi_read_fpga_register(unsigned char reg_idx, unsigned short* reg_val)
{
    int ret = 0;

    p347_fpga_info.spi_tx_buff[0] = (0x5<<24) | (reg_idx << 16);
    p347_fpga_info.spi_tx_buff[1] = 0;
    p347_fpga_info.spi_tx_buff[2] = 0;
    memset(&p347_fpga_info.spi_rx_buff[0],0,SPI_RXBUFF_LEN*sizeof(u32));

    spi_message_init(&p347_fpga_info.msg);

    p347_fpga_info.transfer.len = 3*sizeof(u32); //3 words (cmd,wait,get)
    p347_fpga_info.transfer.cs_change = 1;
    p347_fpga_info.transfer.bits_per_word = 32;
    p347_fpga_info.transfer.speed_hz = p347_fpga_info.new_hz;
    p347_fpga_info.transfer.tx_buf = p347_fpga_info.spi_tx_buff;
    p347_fpga_info.transfer.rx_buf = p347_fpga_info.spi_rx_buff;

    spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);

    //sleep until complete
    //TODO: test, think about hanging up, do something
    ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
    //_PDBA("spi_sync in spi_read_fpga_register ret=%d\n",ret);
    if (ret != 0) return ret;

    *reg_val = (p347_fpga_info.spi_rx_buff[2] & 0x0000FFFF);
    //_PDBA("Reg value = 0x%04x\n",*reg_val);
    //_PDBA("rec[0]=0x%8x, rec[1]=0x%8x, reg_val=0x%4x\n",p347_fpga_info.spi_rx_buff[0],p347_fpga_info.spi_rx_buff[1],*reg_val);

    return 0;
}

int spi_write_adc_register(unsigned char adc_idx, unsigned char reg_idx, unsigned short reg_val)
{
    int ret;
    //request register writing
    p347_fpga_info.spi_tx_buff[0] = (1<<28) | (adc_idx<<24) | (reg_idx<<16) | (reg_val);
    p347_fpga_info.spi_tx_buff[1] = 0; //wait
    //p347_fpga_info.spi_tx_buff[2] = 0;
    //p347_fpga_info.spi_tx_buff[3] = 0;

    spi_message_init(&p347_fpga_info.msg);

    p347_fpga_info.transfer.len = 2*sizeof(u32); //length in bytes
    p347_fpga_info.transfer.cs_change = 1;
    p347_fpga_info.transfer.bits_per_word = 32;
    p347_fpga_info.transfer.speed_hz = p347_fpga_info.new_hz;
    p347_fpga_info.transfer.tx_buf = p347_fpga_info.spi_tx_buff;
    p347_fpga_info.transfer.rx_buf = p347_fpga_info.spi_rx_buff;

    spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);

    //sleep until complete
    //TODO: test, think about hanging up, do something
    ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
    //_PDBA("spi_sync in spi_write_adc_register ret=%d\n",ret);
    if (ret != 0) return ret;

    //and maybe add check

    return 0;
}

int spi_write_fpga_register(unsigned char reg_idx, unsigned short reg_val)
{
    int ret;
    //request register writing
    p347_fpga_info.spi_tx_buff[0] = (1<<28) | (5<<24) | (reg_idx<<16) | (reg_val);
    p347_fpga_info.spi_tx_buff[1] = 0; //wait

    spi_message_init(&p347_fpga_info.msg);

    p347_fpga_info.transfer.len = 2*sizeof(u32); //1 word
    p347_fpga_info.transfer.cs_change = 1;
    p347_fpga_info.transfer.bits_per_word = 32;
    p347_fpga_info.transfer.speed_hz = p347_fpga_info.new_hz;
    p347_fpga_info.transfer.tx_buf = p347_fpga_info.spi_tx_buff;
    p347_fpga_info.transfer.rx_buf = p347_fpga_info.spi_rx_buff;

    spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);

    //sleep until complete
    //TODO: test, think about hanging up, do something
    ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
    //_PDBA("spi_sync in spi_write_fpga_register ret=%d\n",ret);
    if (ret != 0) return ret;

    return 0;
}

//==========================================================================
//==========================================================================
//==========================================================================

//static int p347_fpga_ioctl(struct inode *inode, struct file *filp, uint cmd, unsigned long arg)
static long p347_fpga_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
    t_gpmc_data*	setup_g;
    t_spi_data*		setup_s;
    //t_ch_params*	ch_p;
    //t_transfer_params*	tp;
    //t_data_buffer*	dbuf;
    //t_selfgeneration*	sg;
    long ret_code = 0;
    char kdata[20];
    int i,wcnt,ret,len,p1,p2 = 0;
    unsigned short* us = (unsigned short*)&kdata[0];
    unsigned short reg_idx = 0;
    unsigned short reg_val = 0;
    unsigned short adc_idx;
    //unsigned short tmps = 0;
    //unsigned long flags = 0;
    //unsigned char testdata[10] = {1,2,3,4,5,6,7,8,9,0};
    //siginfo_t usig;
    
    //lock_kernel();
    
    memset(&kdata[0],0,20);
    //_PDBA("p347_fpga_ioctl start: inode=0x%p, filp=0x%p, cmd=%d, arg=0x%p\n",(void*)inode,(void*)filp,cmd,(void*)arg);
    switch (cmd) {
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
		
		spi_message_init(&p347_fpga_info.msg);
		
		ret = copy_from_user(p347_fpga_info.spi_tx_buff,setup_s->data_tx,setup_s->word_cnt*sizeof(u32));
		if (ret > 0) _PDBA("p347_IOCTL_READ: cannot copy %d bytes from userspace\n",ret);
		
		p347_fpga_info.transfer.len = setup_s->word_cnt*sizeof(u32);
		p347_fpga_info.transfer.bits_per_word = 32;
		p347_fpga_info.transfer.speed_hz = p347_fpga_info.new_hz;
		p347_fpga_info.transfer.tx_buf = p347_fpga_info.spi_tx_buff;
		p347_fpga_info.transfer.rx_buf = p347_fpga_info.spi_rx_buff;
		
		_PDBA("rx_buff = 0x%p, tx_buff = 0x%p\n",p347_fpga_info.spi_rx_buff,p347_fpga_info.spi_tx_buff);
		
		spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);
		
		//sleep until complete
		//TODO: test, think about hanging up, do something
		ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
		_PDBA("spi_sync in SPI_SENDCMD ret=%d\n",ret);
		//_PDBA("data_dst = 0x%p, data_src = 0x%p, words = %d\n",setup_s->data_rx,p347_fpga_info.spi_rx_buff,setup_s->word_cnt);
		ret = copy_to_user(setup_s->data_rx,p347_fpga_info.spi_rx_buff,setup_s->word_cnt*sizeof(u32));
		if (ret > 0) _PDBA("p347_IOCTL_READ: cannot copy %d bytes to userspace\n",ret);
		for (wcnt=0; wcnt<setup_s->word_cnt; wcnt++)
		    _PDBA("SPI answer[%d]=0x%x\n",wcnt,p347_fpga_info.spi_rx_buff[wcnt]);
	    }
	break; }
	case p347_IOCTL_SPI_READ_REGISTER: {
	    if (arg == 0) {
		ret_code -p347_ERROR_SPI_INVAL_ARG;
		break;
	    }
	    setup_s = (t_spi_data*)arg;
	    reg_idx = ((unsigned short)(setup_s->data_tx[0]));
	    adc_idx = ((unsigned short)(setup_s->data_tx[1]));
            if ((reg_idx < 1) || (reg_idx > p347_FPGA_REG_TEST_WRITE)) {
		ret_code = -p347_ERROR_SPI_INVAL_REG_NUM;
		break;
	    }

	    if (reg_idx < p347_FPGA_REG_SETTINGS) {//read adc register
		ret = spi_read_adc_register(adc_idx,reg_idx,&reg_val);
	    } else {//read fpga register
		ret = spi_read_fpga_register(reg_idx,&reg_val);
	    }
	    if (ret != 0) {
		ret_code = ret;
		break;
	    }
	
	    reg_val = (p347_fpga_info.spi_rx_buff[1] >> 16);
	    //_PDBA("rec[0]=0x%08x, rec[1]=0x%08x, reg_val=0x%04x\n",p347_fpga_info.spi_rx_buff[0],p347_fpga_info.spi_rx_buff[1],reg_val);

	    ret = copy_to_user(setup_s->data_rx,&reg_val,2);
	    if (ret != 0) {
		ret_code = ret;
		break;
	    }
	break; }
	case p347_IOCTL_SPI_WRITE_REGISTER: {
	    if (arg == 0) return -p347_ERROR_SPI_INVAL_ARG;
	    setup_s = (t_spi_data*)arg;
	    reg_idx = ((unsigned short)(setup_s->data_tx[0]));
	    reg_val = ((unsigned short)(setup_s->data_tx[1]));
            adc_idx = ((unsigned short)(setup_s->data_tx[2]));
            if ((reg_idx < 1) || (reg_idx > p347_FPGA_REG_TEST_WRITE)) {
		ret_code = -p347_ERROR_SPI_INVAL_REG_NUM;
		break;
	    }

	    if (reg_idx < p347_FPGA_REG_SETTINGS) {//write adc register
	        ret = spi_write_adc_register(adc_idx,reg_idx,reg_val);
	    } else {//write fpga register
		ret = spi_write_fpga_register(reg_idx,reg_val);
	    }
	    if (ret != 0) {
	        ret_code = ret;
	        break;
	    }
	break; }
	case p347_IOCTL_SPI_SET_HZ: {
	    if ((arg >= p347_SPI_MIN_SPEED) && (arg <= p347_fpga_info.spidev->max_speed_hz)) {
		_PDBA("Modify spi transfer speed from %ld to %ld\n",p347_fpga_info.new_hz,arg);
		p347_fpga_info.new_hz = arg;
	    } else {
		_PDBA("Incorrect input HZ for SPI, should be between 1.5kHZ and %d\n",p347_fpga_info.spidev->max_speed_hz);
		_PDB("You must use speed definitions from p347_fpga_user.h\n");
	    }
	break; }
	//---------------------------------------------------------------------------------Client
	case p347_IOCTL_GET_INT_CNT: {
	    if (arg>0) {
		put_user(p347_fpga_info.i_cnt,(unsigned long*)arg);
	    }
	break; }
	default: {
	    _PDBA("unknown cmd %d\n",cmd);
	break; }
    };
    //_PDB("p347_fpga_ioctl end\n");
    
    //unlock_kernel();
    return ret_code;
}
/*
static void nl_callback(struct sk_buff *__skb)
{
    printk("p347 nl_callback\n");
}
*/
//==========================================================================

static const struct file_operations p347_fpga_fops = {
    .owner		= THIS_MODULE,
    .unlocked_ioctl	= p347_fpga_ioctl,
    .open		= p347_fpga_open,
    .release		= p347_fpga_release,
    .fasync		= NULL,
    .llseek		= no_llseek,
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
    spi->mode = SPI_MODE_0;
    ret = spi_setup(spi);
    _PDBA("spi_setup ret %d\n",ret);
    
    //update MCSPI_SYST register
    __raw_writel(0x00000100,ioremap(0x480B8000,SZ_4K)+0x24);
    
    _PDBA("master->mode=%x, master->flags=%x, spi->dev initname = %s\n",
	spi->master->mode_bits,spi->master->flags,spi->dev.init_name);
    
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

static int p347_spi_suspend(struct spi_device *spi, pm_message_t message)
{
    _PDBA("p347_spi_suspend, message=%d\n",message.event);
    return 0;
}

static int p347_spi_resume(struct spi_device *spi)
{
    _PDB("p347_spi_resume\n");
    return 0;
}

//==========================================================================

static struct spi_driver p347_spi_driver = {
    .driver = {
	.name 	= "p347_spi",
	.bus 	= &spi_bus_type,
	.owner 	= THIS_MODULE,
    },
    .probe 	= p347_spi_probe,
    .remove	= __devexit_p(p347_spi_remove),
    .suspend	= p347_spi_suspend,
    .resume	= p347_spi_resume,
};

//==========================================================================
//==========================================================================
//==========================================================================

//less print
#define psp_padconf(addr,offset,value)\
    padconf_addr = scm_base + addr;\
    reg_val = __raw_readl(padconf_addr);\
    if (offset) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;\
    reg_val |= (value << offset);\
    __raw_writel(reg_val,padconf_addr)

static int __init p347_fpga_init(void)
{
    int ret,i;
    u32 reg_val,old;
    size_t tmp_sz = 0;
    unsigned long rate;
    unsigned long scm_base = (unsigned long)ioremap(0x48000000,SZ_4K);
    unsigned long cm_addr = scm_base + 0x4d40;
    unsigned long padconf_addr;
#ifdef YURI_BOARD
    //unsigned long gpio_base = (unsigned long)ioremap(0x48310000,SZ_4K); //GPIO bank 1
    unsigned long gpio_base = (unsigned long)ioremap(0x49054000,SZ_4K); //GPIO bank 4
#else
    unsigned long gpio_base = (unsigned long)ioremap(0x49054000,SZ_4K); //GPIO bank 4
#endif
    unsigned long dmareg_base = (unsigned long)ioremap(0x48056000,SZ_4K);
    unsigned long mpuintc_base = (unsigned long)ioremap(0x48200000,SZ_4K);
    unsigned long tmp_addr;
    
    spin_lock_init(&irq_lock);

    _PDB("p347_fpga_init start\n");

    memset(&p347_fpga_info,0,sizeof(p347_fpga_device_t));
    p347_fpga_info.i_cnt = 0;

    //-----------------------------------------------------------memory allocations
    
    tmp_sz = SPI_RXBUFF_LEN*sizeof(u32);
    p347_fpga_info.spi_rx_buff = kzalloc(tmp_sz,GFP_KERNEL);
    if (p347_fpga_info.spi_rx_buff == NULL) {
	_PDBA("Cannot allocate space for spi_rx_buff (%d bytes)\n",tmp_sz);
	return -ENOMEM;
    }
    _PDBA("spi_rx_buff kzallocated in 0x%p\n",p347_fpga_info.spi_rx_buff);
    
    tmp_sz = SPI_TXBUFF_LEN*sizeof(u32);
    p347_fpga_info.spi_tx_buff = kzalloc(tmp_sz,GFP_KERNEL);
    if (p347_fpga_info.spi_tx_buff == NULL) {
	_PDBA("Cannot allocate space for spi_tx_buff (%d bytes)\n",tmp_sz);
	kfree(p347_fpga_info.spi_rx_buff);
	return -ENOMEM;
    }
    _PDBA("spi_tx_buff kzallocated in 0x%p\n",p347_fpga_info.spi_tx_buff);
    
    //-----------------------------------------------------------fpga_irq pad configuration
    padconf_addr = scm_base + PADCONF_FPGA_IRQ_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    if (PADCONF_FPGA_IRQ_OFFSET) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;
    reg_val |= (PADCONF_FPGA_IRQ_VALUE << PADCONF_FPGA_IRQ_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);

    //-----------------------------------------------------------mcspi3 pad configuration
    psp_padconf(PADCONF_SPI3_SIMO_ADDR,PADCONF_SPI3_SIMO_OFFSET,PADCONF_SPI3_SIMO_VALUE);
    psp_padconf(PADCONF_SPI3_SOMI_ADDR,PADCONF_SPI3_SOMI_OFFSET,PADCONF_SPI3_SOMI_VALUE);
    psp_padconf(PADCONF_SPI3_CS1_ADDR,PADCONF_SPI3_CS1_OFFSET,PADCONF_SPI3_CS1_VALUE);
    psp_padconf(PADCONF_SPI3_CLK_ADDR,PADCONF_SPI3_CLK_OFFSET,PADCONF_SPI3_CLK_VALUE);
    psp_padconf(PADCONF_SPI3_CS0_ADDR,PADCONF_SPI3_CS0_OFFSET,PADCONF_SPI3_CS0_VALUE);
    
    //-----------------------------------------------------------dpll3 clock divider (make low rate)
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
    
    p347_fpga_info.irqnum = gpio_to_irq(p347_GPIO_FPGA_IRQ);
    _PDBA("gpio_to_irq ret irqnum=%d\n",p347_fpga_info.irqnum);
    
    ret = request_irq(p347_fpga_info.irqnum,p347_fpga_irq_handler,0,"p347_fpga_irq",&p347_fpga_info);
    _PDBA("request_irq %d ret %d\n",p347_fpga_info.irqnum,ret);

    old = __raw_readl(gpio_base + 0x30); //GPIO_CTRL
    reg_val = 0; //enable module, no gating
    __raw_writel(reg_val,gpio_base + 0x30);
    reg_val = __raw_readl(gpio_base + 0x30);
    _PDBA("CTRL for GPIO old=0x%08x, new=0x%08x\n",old,reg_val);

    old = __raw_readl(gpio_base + 0x10); //GPIO_SYSCONFIG
    reg_val = 0x8; //No-idle mode
    __raw_writel(reg_val,gpio_base + 0x10);
    reg_val = __raw_readl(gpio_base + 0x10);
    _PDBA("SYSCONFIG for GPIO old=0x%08x, new=0x%08x\n",old,reg_val);
    
    __raw_writel(0,gpio_base + 0x20); //no wakeup enable
    
    old = __raw_readl(gpio_base + 0x34); //GPIO_OE
    reg_val = old | (1 << p347_GPIO_BITPOS); //set as input
    __raw_writel(reg_val,gpio_base + 0x34);
    reg_val = __raw_readl(gpio_base + 0x34);
    _PDBA("OE for GPIO old=0x%08x, new=0x%08x\n",old,reg_val);

    old = __raw_readl(gpio_base + 0x50); //GPIO_DEBOUNCEENABLE
    //reg_val = old & (~(1 << p347_GPIO_BITPOS)); //No debounce
    reg_val = old & (1 << p347_GPIO_BITPOS); //yes
    __raw_writel(reg_val,gpio_base + 0x50);
    reg_val = __raw_readl(gpio_base + 0x50);
    _PDBA("DEBOUNCEENABLE for GPIO old=0x%08x, new=0x%08x\n",old,reg_val);
/*
    reg_val = __raw_readl(gpio_base + 0x4048); //GPIO4 RISINGDETECT
    reg_val |= (1 << p347_GPIO_BITPOS);
    __raw_writel(reg_val,gpio_base + 0x4048);
*/
    reg_val = __raw_readl(gpio_base + 0x4C); //GPIO4 FALLINGDETECT
    reg_val |= (1 << p347_GPIO_BITPOS);
    __raw_writel(reg_val,gpio_base + 0x4C);

/*
    reg_val = __raw_readl(gpio_base + 0x4014); //GPIO_SYSSTATUS
    _PDBA("SYSSTATUS for GPIO4 = 0x%08x\n",reg_val);
    reg_val = __raw_readl(gpio_base + 0x4038); //GPIO_DATAIN
    _PDBA("DATAIN for GPIO4 = 0x%08x\n",reg_val);
*/    
    old = __raw_readl(gpio_base + 0x1C); //GPIO_IRQENABLE1
    reg_val = old | (1 << p347_GPIO_BITPOS);
    __raw_writel(reg_val,gpio_base + 0x1C);
    reg_val = __raw_readl(gpio_base + 0x1C);
    _PDBA("IRQENABLE1 for GPIO old=0x%08x, new=0x%08x\n",old,reg_val);

#ifdef CS67_MODE
    //request cs6 and cs7 as gpio
    ret = gpio_request(p347_GPIO_OSCIRQ,"gpio_oscirq");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_oscirq, ret=%d\n",ret);
	return ret;
    }
    ret = gpio_request(p347_GPIO_OSCDMA,"gpio_oscdma");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_oscdma, ret=%d\n",ret);
	return ret;
    }

    //pad configuration for CS6
    padconf_addr = scm_base + PADCONF_OSCIRQ;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0x0000FFFF;
    reg_val |= (PADCONF_OSCIRQ_VALUE << PADCONF_OSCIRQ_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    //pad configuration for CS7
    padconf_addr = scm_base + PADCONF_OSCDMA;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0xFFFF0000;
    reg_val |= PADCONF_OSCDMA_VALUE;
    __raw_writel(reg_val,padconf_addr);
    
    gpio_direction_output(p347_GPIO_OSCIRQ,1);
    gpio_direction_output(p347_GPIO_OSCDMA,1);
#endif
#ifdef YURI_BOARD
    //request qpio for fpga reset
    ret = gpio_request(p347_GPIO_FPGA_RESET,"gpio_fpga_reset");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_fpga_reset, ret=%d\n",ret);
	return ret;
    }
    
    padconf_addr = scm_base + PADCONF_FPGA_RESET_ADDR;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0x0000FFFF;
    reg_val |= (PADCONF_FPGA_RESET_VALUE << PADCONF_FPGA_RESET_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    
    gpio_direction_output(p347_GPIO_FPGA_RESET,1);
#endif

    ret = register_chrdev(p347_CHARDEV_NUM,p347_CHARDEV_NAME,&p347_fpga_fops);
    if (ret < 0) {
	//gpio_free(p347_GPIO_FPGA_CS);
	gpio_free(p347_GPIO_FPGA_IRQ);
	_PDB("ERROR: cannot register char device for p347_fpga\n");
	return ret;
    }

    padconf_addr = scm_base + PADCONF_FPGA_IRQ_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    //-----------------------------------------------------------
    //TODO: maybe move to board description
    ret = spi_register_driver(&p347_spi_driver);
    _PDBA("spi_register_driver ret=%d\n",ret);
    //-----------------------------------------------------------

    //print_gpmc_padconf(scm_base);
    //print_clken_registers();

    //spin_lock_init(&(tpi.ROT.lock));

    printk("FPGA_RESET=>1\n");
    gpio_set_value(p347_GPIO_FPGA_RESET,1);

    _PDB("p347_fpga_init end\n");
    return 0;
}
device_initcall(p347_fpga_init);

//==========================================================================

static void __exit p347_fpga_exit(void)
{
    int i;
    _PDB("p347_fpga_exit start\n");

    spi_unregister_driver(&p347_spi_driver);

    unregister_chrdev(p347_CHARDEV_NUM,p347_CHARDEV_NAME);

    free_irq(p347_fpga_info.irqnum,&p347_fpga_info);

    //gpio_free(p347_GPIO_FPGA_CS);
    gpio_free(p347_GPIO_FPGA_IRQ);
    //gpmc_cs_free(p347_GPMC_FPGA_CS_READ);
#ifdef CS67_MODE
    gpio_free(p347_GPIO_OSCIRQ);
    gpio_free(p347_GPIO_OSCDMA);
#else
    //gpmc_cs_free(p347_GPMC_FPGA_CS_WRITE);
#endif
    kfree(p347_fpga_info.spi_rx_buff);
    kfree(p347_fpga_info.spi_tx_buff);

    //platform_device_unregister(p347_fpga_info->pdev);
    _PDB("p347_fpga_exit end\n");
}

//==========================================================================
//==========================================================================
//==========================================================================
//module_init(p347_fpga_init);
module_exit(p347_fpga_exit);
//==========================================================================
MODULE_AUTHOR("Konstantin Utkin");
MODULE_LICENSE("GPL");
//==========================================================================
