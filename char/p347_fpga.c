/*
    p347 test fpga driver
    
    uses GPMC, SPI, GPIO
    uses _PDB instead of printk for detailed information
*/
/*
    Global TODO:
    1) Use only specific ( p347_ ) error codes

*/

#define PROGRAM_AUTHOR			"Konstantin Utkin"
#define p347_DRV_VERSION		"p347 FPGA driver " __DATE__ " " __TIME__ " by " PROGRAM_AUTHOR

//#define USE_FIQ
#undef USE_FIQ

//#define VERIFY_REGISTERS
#undef VERIFY_REGISTERS

//#define USE_IRQSAVE_IN_IRQ
//#define USE_IRQSAVE_IN_CALLBACK
//#define USE_HI_TASKLET

#include <linux/p347_fpga.h>
#include <asm/cacheflush.h>

#ifdef USE_HI_TASKLET
static struct tasklet_struct irq_delayed_task;
#endif

#ifdef USE_FIQ
#include <asm/fiq.h>

static struct fiq_handler fh = {
    .name		= "p347_fpga_irq",
};
static int fh_id = 0;
/*
static void p347_fiq_handler(void) {
    asm volatile(".global p347_fiq_start\n p347_fiq_start:");

    // clear gpio irq
    asm("ldr r10, GPIO_BASE_ISR");
    asm("ldr r9, [r10]");
    asm("orr r9, #0x04");
    asm("str r9, [r10]");

    //increment int_cnt
    
    //check if dma active or not
    
    //if active, increment delay_counter
    
    //if not active, start dma and set gpio

    asm(".global p347_fiq_end\n p347_fiq_end:");
}
*/
#endif

#define GPMC_CS0            0x60
#define GPMC_CS_SIZE		0x30


#define FPGA_LOAD_WAIT_MSEC     500
#define FPGA_LOAD_WAIT_CYCLES   4

#define DP(_x_) if (tpi.deb_irq_print) printk(_x_)

//#define SPI_RECEIVE

static void* d_ret = NULL;

static p347_fpga_device_t	p347_fpga_info;

//static struct gpmc_timings g_time;
static unsigned long fpga_mem_read_base;
static unsigned long fpga_mem_write_base;
static unsigned long fpga_mem_read_addr;
static unsigned long fpga_mem_write_addr;

static void __iomem *gpmc_base;// = 0x6e000000;
static struct clk *gpmc_l3_clk = NULL;
static struct clk *dpll3_clk = NULL;

static spinlock_t irq_lock;
static spinlock_t dma_lock;
//static const unsigned short GPMC_MHZ = 80;
static const unsigned short GPMC_MHZ = 166;

static t_p347_inner	tpi;
static unsigned long	sul_icnt = 0;

static iii = 0;

#define p347_DMA_DIR	DMA_FROM_DEVICE
//#define p347_DMA_DIR	DMA_TO_DEVICE
//#define p347_DMA_DIR	DMA_BIDIRECTIONAL

//#define p347_DMA_SIZE	0xA00000
#define p347_DMA_SIZE	4*FRAMES_FOR_SINGLE_CHANNEL

int _create_adc_buffer(t_data_buffer *dbuf) {
    if (!dbuf) return -EINVAL;
    memset(dbuf,0,sizeof(t_data_buffer));
#ifdef DMA_DIRECT
    
    dbuf->ptr = phys_to_virt(0x9a000000+iii*p347_PUMPBUF_BYTESIZE);
    dbuf->dma_base = 0x9a000000+iii*p347_PUMPBUF_BYTESIZE;
    printk("set pointers base=%08p, virt=%08p\n",dbuf->dma_base,dbuf->ptr);
    
    iii++;
#else
    dbuf->ptr = kcalloc(p347_PUMPBUF_FULLSIZE,sizeof(u16),GFP_KERNEL);
    if (!dbuf->ptr) {
	    printk("ERROR: cannot allocate buffer for ADC channel\n");
	    return -ENOMEM;
    }
#endif
    spin_lock_init(&(dbuf->lock));
    
    _PDBA("ADC buffer created: wpos=%d, rpos=%d, cur_len=%d\n",
	atomic_read(&(dbuf->write_pos)),atomic_read(&(dbuf->read_pos)),atomic_read(&(dbuf->cur_len)) );
    
    return 0;
}

void _delete_adc_buffer(t_data_buffer *dbuf) {
    if (!dbuf) return;
    if (dbuf->ptr) {
#ifdef DMA_DIRECT
        //dma_free_coherent(NULL,p347_PUMPBUF_BYTESIZE,dbuf->ptr,dbuf->dma_base);
	//dma_unmap_single(NULL,dbuf->dma_base,0xA00000,DMA_FROM_DEVICE);
	//dma_unmap_single(NULL,dbuf->dma_base,0xA00000,p347_DMA_DIR);
#else
	kfree(dbuf->ptr);
#endif
    }
    memset(dbuf,0,sizeof(t_data_buffer));
}

int _create_timestamps_buffer(t_timestamps_buffer *rbuf) {
    if (!rbuf) return -EINVAL;
    memset(rbuf,0,sizeof(t_timestamps_buffer));
    
#ifdef DMA_DIRECT
    rbuf->ptr = dma_alloc_coherent(NULL,p347_PUMPBUF_BYTESIZE,&rbuf->dma_base,GFP_DMA);
    if (!rbuf->ptr) {
	    printk("ERROR: cannot allocate DMA buffer for TIMESTAMPS\n");
	    return -ENOMEM;
    }
#else
    rbuf->ptr = kcalloc(p347_PUMPBUF_FULLSIZE,sizeof(u16),GFP_KERNEL);
    if (!rbuf->ptr) {
	    printk("ERROR: cannot allocate buffer for TIMESTAMPS\n");
	    return -ENOMEM;
    }
#endif
    _PDBA("TIMESTAMPS buffer created: wpos=%d, rpos=%d, cur_len=%d\n",
	atomic_read(&(rbuf->write_pos)),atomic_read(&(rbuf->read_pos)),atomic_read(&(rbuf->cur_len)) );
    
    return 0;
}

void _delete_timestamps_buffer(t_timestamps_buffer *rbuf) {
    if (!rbuf) return;
    if (rbuf->ptr) {
#ifdef DMA_DIRECT
        dma_free_coherent(NULL,p347_PUMPBUF_MAXRECORDS*4,rbuf->ptr,rbuf->dma_base);
#else
	kfree(rbuf->ptr);
#endif
    }
    memset(rbuf,0,sizeof(t_timestamps_buffer));
}

//==========================================================================
//==========================================================================		GPMC
//==========================================================================

u32 my_gpmc_cs_read_reg(int cs, int idx)
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
//==========================================================================		GPMC
//==========================================================================

//TODO: maybe user should know which register exactly causes error
#define TRY_SET(_idx_,_val_) \
    ret = spi_write_adc_register(adc_idx,_idx_,_val_);\
    if (ret != 0) return ret

#define TRY_SET_FPGA(_idx_,_val_)\
    ret = spi_write_fpga_register(_idx_,_val_);\
    if (ret != 0) return ret

#define IS_CH_RUNNING(_idx_)\
    (atomic_read(&(tpi.ABUF[_idx_].is_running)))

#define IS_ROT_RUNNING(_idx_)\
    (atomic_read(&(tpi.is_rot_running[_idx_])))

#ifdef SPI_RECEIVE

static struct delayed_work spi_work;

void delayed_function( unsigned long data );
//DECLARE_TASKLET(p347_spi_tasklet, tasklet_function, NULL);
#endif

#ifdef SPI_RECEIVE
static irqreturn_t p347_fpga_irq_handler(int irq, void* data)
{
    schedule_delayed_work(&spi_work,msecs_to_jiffies(10));
    gpio_set_value(p347_GPIO_OSCIRQ,0);
    return IRQ_HANDLED;
}
#endif

#ifdef USE_HI_TASKLET
static void do_hi_tasklet(unsigned long data)
{
    int i;
    
    t_data_buffer* _abuf_;
    dma_addr_t dat;

	for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	    _abuf_ = &(tpi.ABUF[i]);
	    //invalidate_kernel_vmap_range(_abuf_->ptr+atomic_read(&_abuf_->write_pos),p347_DMA_SIZE);
	    dat = dma_map_single(NULL,(_abuf_->ptr+atomic_read(&_abuf_->write_pos)),p347_DMA_SIZE,p347_DMA_DIR);
	    //invalidate_kernel_vmap_range(_abuf_->ptr+atomic_read(&_abuf_->write_pos),p347_DMA_SIZE);
	    
	    //dma_sync_single_for_device(NULL,dat,p347_DMA_SIZE,p347_DMA_DIR);
	    
	    //printk("CH%d new base %08p virt %08p\n",i,dat,_abuf_->ptr+atomic_read(&_abuf_->write_pos));
	    //printk("dst_addr = %08p\n",p347_fpga_info.dchain[i].dst_start);
	}

	omap_start_dma(p347_fpga_info.chain_channel[0]);
}
#endif

static unsigned long irqstatus_reg_addr;
static unsigned long gpio_data_reg_addr;
static unsigned long gpio_inen_reg_addr;
static unsigned long ipend0;
static unsigned long ipend1;
static unsigned long ipend2;

static irqreturn_t p347_fpga_irq_handler(int irq, void* data)
{
#ifdef USE_IRQSAVE_IN_IRQ
    unsigned long flags;
#endif
    //unsigned long i1,i2,i3;
    //int i;
    
    //_PDB("p347 IRQ!\n");
    /*
    flags =__raw_readl(irqstatus_reg_addr);
    _PDBA("irq flags = 0x%08x\n",flags);
    flags =__raw_readl(gpio_data_reg_addr);
    _PDBA("irq flags = 0x%08x\n",flags);
    flags =__raw_readl(gpio_inen_reg_addr);
    _PDBA("irq flags = 0x%08x\n",flags);
    */
    
    //i1 = __raw_readl(ipend0);
    //i2 = __raw_readl(ipend1);
    //i3 = __raw_readl(ipend2);
    //_PDBA("ipend0 =0x%08x, ipend1 =0x%08x, ipend2 =0x%08x\n",i1,i2,i3);
    
    //if ((i2 && 0x1) == 0) return IRQ_NONE;
    
    //if ((flags && p347_GPIO_BITPOS) == 0)
	//_PDB("Fake interrupt!\n");
    /*
    unsigned long flags;
    spin_lock_irqsave(&irq_lock,flags);
    sul_icnt++;
    spin_unlock_irqrestore(&irq_lock,flags);
    */

    //gpio_set_value(p347_GPIO_OSCIRQ,1-gpio_get_value(p347_GPIO_OSCIRQ));
    //return IRQ_HANDLED;

    //p347_fpga_info.i_cnt++; //count interrupts
#ifdef USE_HI_TASKLET
    atomic_inc(&p347_fpga_info.i_cnt);

    if (atomic_read(&tpi.dma_used) != 0) {
	atomic_inc(&tpi.dma_delayed);
    } else {
	atomic_set(&tpi.dma_used,1);
	tasklet_hi_schedule(&irq_delayed_task);
    }
    //if (printk_ratelimit())
//	printk("schedule tasklet\n");
#else
    t_data_buffer* _abuf_;
    dma_addr_t dat;
    int i;

    atomic_inc(&p347_fpga_info.i_cnt);

    if (atomic_read(&tpi.dma_used) != 0) {
	atomic_inc(&tpi.dma_delayed);
    } else {
	atomic_set(&tpi.dma_used,1);
#ifdef USE_IRQSAVE_IN_IRQ
	spin_lock_irqsave(&irq_lock,flags);
#endif
	gpio_set_value(p347_GPIO_OSCIRQ,0);
	/*
	for (i=0; i<p347_ADC_CHANNELS_COUNT; i++) {
	    p347_fpga_info.dchain[i].dst_start = tpi.ABUF[i].dma_cur;
	    omap_set_dma_params(p347_fpga_info.chain_channel[i],&p347_fpga_info.dchain[i]);
	}
	*/
	for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	    _abuf_ = &(tpi.ABUF[i]);
	    dat = dma_map_single(NULL,(_abuf_->ptr+atomic_read(&_abuf_->write_pos)),p347_DMA_SIZE,p347_DMA_DIR);
	}
	
	omap_start_dma(p347_fpga_info.chain_channel[0]);
#ifdef USE_IRQSAVE_IN_IRQ
        spin_unlock_irqrestore(&irq_lock,flags);
#endif
    }
#endif
    //spin_unlock_irqrestore(&irq_lock,flags);

    return IRQ_HANDLED;
}

inline unsigned long calc_crc_32(void) {
    return ( ((unsigned short*)(d_ret))[0]+((unsigned short*)(d_ret))[1]+
	     ((unsigned short*)(d_ret))[2]+((unsigned short*)(d_ret))[3]+((unsigned short*)(d_ret))[4]);
}

static unsigned short* tptr;
static unsigned short* dptr;

static void p347_fpga_dma_callback(int lch, u16 ch_status, void* data)
{
    int dd = 0;
    int i;
    dma_addr_t dat;
    unsigned char r_cnt = 0;
    unsigned short shift_value;
    unsigned long tmpl;
#ifdef USE_IRQSAVE_IN_CALLBACK
    unsigned long flags;
#endif
    int tmpi = 0;
    t_data_buffer* _abuf_;
    
    //if (printk_ratelimit())
//	_PDB("dma_callback\n");

    if (p347_fpga_info.chain_channel[p347_ADC_CHANNELS_CNT-1] != lch) {
	_PDBA("ERROR! dma lch=%d, should be %d\n",lch,p347_fpga_info.chain_channel[p347_ADC_CHANNELS_CNT-1]);
	return;
    }
    if (ch_status & ~OMAP_DMA_BLOCK_IRQ) {
	_PDBA("ERROR! not a block irq!, ch_status = 0x%04x\n",ch_status);
	return;
    }
    if (ch_status & 0x0F00) {
	_PDBA("ATTENTION! ch_status = 0x%04x\n",ch_status);
	return;
    }
    
    omap_stop_dma(p347_fpga_info.chain_channel[p347_ADC_CHANNELS_CNT-1]);
#ifdef USE_IRQSAVE_IN_CALLBACK
    spin_lock_irqsave(&dma_lock,flags);
#endif
    //flush_cache_all();
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
        _abuf_ = &(tpi.ABUF[i]);
        //printk("unmap _abuf_->dma_cur = %08p\n",_abuf_->dma_cur);
        dma_unmap_single(NULL,_abuf_->dma_cur,p347_DMA_SIZE,p347_DMA_DIR);
	
	//dma_sync_single_for_cpu(NULL,_abuf_->dma_cur,p347_DMA_SIZE,p347_DMA_DIR);
	//flush_kernel_vmap_range(_abuf_->ptr+atomic_read(&(_abuf_->write_pos)),p347_DMA_SIZE);
	
	if (IS_CH_RUNNING(i)) {
	    r_cnt++;
	    //_PDBA("CH%d work\n",i);
	    if ((atomic_read(&(_abuf_->proc_len)) + 2*DATA_FRAME_SIZE) > p347_PUMPBUF_FULLSIZE) {
        	atomic_set(&(_abuf_->overflow),1);
	    } else { //update
    		//dma_unmap_single(NULL,_abuf_->dma_cur,4*FRAMES_FOR_SINGLE_CHANNEL,p347_DMA_DIR);
    	    
    		atomic_set(&(_abuf_->overflow),0);
    		//modify pointers
    		tmpi = atomic_read(&(_abuf_->write_pos));
    		//_PDBA("tmpi=%d\n",tmpi);
    		//default mode
    		tmpi += 2*DATA_FRAME_SIZE;
		atomic_add(2*DATA_FRAME_SIZE,&(_abuf_->cur_len));
		atomic_add(2*DATA_FRAME_SIZE,&(_abuf_->proc_len));
    		
    		/*
    		//missing word-shift mode
    		tmpi += 2*(DATA_FRAME_SIZE-1);
    		shift_value = *(_abuf_->ptr + tmpi);
    		tmpl = *((unsigned long*)(_abuf_->ptr + tmpi));
    		if (shift_value != 0) {
    		    if (printk_ratelimit())
    			_PDBA("CH%d incomplete pack, shift =0x%04x, tmpl = 0x%08x\n",i,shift_value,tmpl);
    		    tmpi -= 2*shift_value;
    		}
    		
    		if (shift_value >= DATA_FRAME_SIZE) { //invalid value
    		    shift_value = 2*DATA_FRAME_SIZE; //default
    		    tmpi = atomic_read(&(_abuf_->write_pos)) + 2*DATA_FRAME_SIZE;
    		} else
    		    shift_value = 2*(DATA_FRAME_SIZE - shift_value - 1);
    		
    		atomic_add(shift_value,&(_abuf_->cur_len));
		atomic_add(shift_value,&(_abuf_->proc_len));
    		//end of missing mode
    		*/
    		
    		if (tmpi >= p347_PUMPBUF_FULLSIZE) tmpi-=p347_PUMPBUF_FULLSIZE;
		atomic_set(&(_abuf_->write_pos),tmpi);
    		
    		//modify dma addr
    		_abuf_->dma_cur = _abuf_->dma_base + tmpi*2;
    		p347_fpga_info.dchain[i].dst_start = _abuf_->dma_cur;
    		p347_fpga_info.dchain[i].src_start = fpga_mem_read_base;
	    }
	}
    }
    
    //omap_stop_dma(p347_fpga_info.chain_channel[p347_ADC_CHANNELS_CNT-1]);
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	//_abuf_ = &(tpi.ABUF[i]);
	//dma_map_single(NULL,_abuf_->ptr+atomic_read(&_abuf_->write_pos),4*FRAMES_FOR_SINGLE_CHANNEL,p347_DMA_DIR);
	
	omap_set_dma_params(p347_fpga_info.chain_channel[i],&p347_fpga_info.dchain[i]);
    }
#ifdef USE_IRQSAVE_IN_CALLBACK
    spin_unlock_irqrestore(&dma_lock,flags);
#endif
    
    /*
    if (printk_ratelimit())
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++) if (IS_CH_RUNNING(i)) {
	_PDBA("CH %d wpos=%d, cur_len=%d\n",i,atomic_read(&(_abuf_->write_pos)),atomic_read(&(_abuf_->cur_len)));
    }
    */
    dd = atomic_read(&tpi.dma_delayed);
    if (dd > 0) {
	atomic_dec(&tpi.dma_delayed);
	//flush_cache_all();
	for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	    _abuf_ = &(tpi.ABUF[i]);
            //invalidate_kernel_vmap_range(_abuf_->ptr+atomic_read(&_abuf_->write_pos),p347_DMA_SIZE);
	    dat = dma_map_single(NULL,_abuf_->ptr+atomic_read(&_abuf_->write_pos),p347_DMA_SIZE,p347_DMA_DIR);
	    //invalidate_kernel_vmap_range(_abuf_->ptr+atomic_read(&_abuf_->write_pos),p347_DMA_SIZE);
	    
	    //dma_sync_single_for_device(NULL,dat,p347_DMA_SIZE,p347_DMA_DIR);
	    //printk("delayed CH%d new base %08p virt %08p\n",i,dat,_abuf_->ptr+atomic_read(&_abuf_->write_pos));
	}
	omap_start_dma(p347_fpga_info.chain_channel[0]);
    } else {
	atomic_set(&tpi.dma_used,0);
	gpio_set_value(p347_GPIO_OSCIRQ,1);
    }

    if ((dd > 3) && printk_ratelimit())
	_PDBA("DMA DELAYED COUNTER = %d\n",dd);
	
    if (r_cnt == 0) {
	if (printk_ratelimit()) {
	    _PDB("IRQ AND DMA WHILE NO CHANNELS RUNNING\n");
	}
    }
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
    //p347_fpga_info.spi_tx_buff[2] = 0;
    //p347_fpga_info.spi_tx_buff[3] = 0;
    //p347_fpga_info.spi_tx_buff[4] = 0;
    memset(&p347_fpga_info.spi_rx_buff[0],0,SPI_RXBUFF_LEN*sizeof(u32));

    spi_message_init(&p347_fpga_info.msg);      
    
    //p347_fpga_info.transfer.len = 5*sizeof(u32); //5 words (cmd,wait,wait,wait,get)
    p347_fpga_info.transfer.len = 2*sizeof(u32);
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
    *reg_val = (p347_fpga_info.spi_rx_buff[1] & 0x0000FFFF);
    //_PDBA("Reg value = 0x%04x\n",*reg_val);

    return 0;
}

int spi_read_fpga32(unsigned char reg_idx, unsigned long* reg_val)
{
    int ret = 0;

    p347_fpga_info.spi_tx_buff[0] = (0x5<<24) | (reg_idx << 16);
    p347_fpga_info.spi_tx_buff[1] = 0;
    //p347_fpga_info.spi_tx_buff[2] = 0;
    memset(&p347_fpga_info.spi_rx_buff[0],0,SPI_RXBUFF_LEN*sizeof(u32));

    spi_message_init(&p347_fpga_info.msg);

    p347_fpga_info.transfer.len = 2*sizeof(u32); //3 words (cmd,wait,get)
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

    *reg_val = p347_fpga_info.spi_rx_buff[1];
    //_PDBA("Reg value = 0x%08lx\n",p347_fpga_info.spi_rx_buff[2]);
    //_PDBA("rec[0]=0x%8x, rec[1]=0x%8x, reg_val=0x%4x\n",p347_fpga_info.spi_rx_buff[0],p347_fpga_info.spi_rx_buff[1],*reg_val);

    return 0;
}

int spi_read_fpga_register(unsigned char reg_idx, unsigned short* reg_val) {
    unsigned long v=0;
    int ret = spi_read_fpga32(reg_idx,&v);
    *reg_val = v & 0xFFFF;
    return ret;
}

#define SPI_MINIMUM_LEN_FOR_DMA		8
#define DMA_BUFFER_LEN_U32		100

static void spi_complete(unsigned long* data) {
    printk("spi_complete\n");
}

int load_signal_to_fpga(unsigned long len, unsigned long* u_ptr) {
    unsigned long datacounter = len;
    unsigned long to_write;
    unsigned long *b_in;
    unsigned long *b_out;
    int ret = 0;
            
            _PDBA("load_signal len=%ld, ptr=%8p\n",len,u_ptr);
            
            //b_out = kzalloc(len*4,GFP_KERNEL);
            //n_in = kzalloc(len*4,GFP_KERNEL);
            b_out = dma_alloc_coherent(NULL,len*4,&p347_fpga_info.spi_tx_dma,GFP_DMA);
            b_in = dma_alloc_coherent(NULL,len*4,&p347_fpga_info.spi_rx_dma,GFP_DMA);
            
            //common setup
	    p347_fpga_info.transfer.cs_change = 1;
	    p347_fpga_info.transfer.bits_per_word = 32;
	    p347_fpga_info.transfer.tx_dma = &p347_fpga_info.spi_tx_dma;
	    p347_fpga_info.transfer.rx_dma = &p347_fpga_info.spi_rx_dma;
	    p347_fpga_info.transfer.tx_buf = b_out;
	    p347_fpga_info.transfer.rx_buf = b_in;

    do {
	
	spi_message_init(&p347_fpga_info.msg);
	//p347_fpga_info.msg.complete = spi_complete;
    
	if (datacounter >= SPI_MINIMUM_LEN_FOR_DMA) { //------DMA for main part of singnal
	    if (datacounter > DMA_BUFFER_LEN_U32) {
		to_write = DMA_BUFFER_LEN_U32;
	    } else {
	        to_write = datacounter;
	    }
            //_PDBA("to_write=%d\n",to_write);
	    copy_from_user(b_out,u_ptr+len-datacounter,to_write); //copy data for transfer
	    p347_fpga_info.transfer.len = to_write*sizeof(u32);
	    //p347_fpga_info.msg.is_dma_mapped = 1;
	    
	    spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);
	    //_PDB("before sync\n");
	    ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
	    //ret = spi_async(p347_fpga_info.spidev,&p347_fpga_info.msg);
	    if (ret != 0) {
	        _PDBA("spi_sync rets with %d\n",ret);
		break;
	    }
	
	    //msleep(100);
	
	    datacounter -= to_write;
	} else { //--------------------------------------------NODMA for tail
	    copy_from_user(b_out,u_ptr+len-datacounter,datacounter); //copy data for transfer
	    p347_fpga_info.transfer.len = datacounter*sizeof(u32);
	    p347_fpga_info.msg.is_dma_mapped = 0;
	    
	    spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);
	    ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
	    if (ret != 0) break;
	    
	    datacounter = 0;
	}

    } while (datacounter>0);

    if (datacounter == 0)
	_PDB("Signal uploading complete\n");

    dma_free_coherent(NULL,len*4,b_out,p347_fpga_info.spi_tx_dma);
    dma_free_coherent(NULL,len*4,b_in,p347_fpga_info.spi_rx_dma);

    return ret;
}

#ifdef SPI_RECEIVE

#define SPI_TRANSFER_LEN		2*DATA_FRAME_SIZE+1

void delayed_function( unsigned long data ) {
    int i;
    int ret;

    unsigned long spi_in[2];
    unsigned long spi_out[2];
    
    for (i=0; i<SPI_TRANSFER_LEN; i++) {
	spi_out[0] = (0x5 << 24) | (0x0b << 16) | i;
	spi_out[1] = 0;
	spi_in[0] = 0;
	spi_in[1] = 0;
	
	spi_message_init(&p347_fpga_info.msg);

	p347_fpga_info.transfer.len = 2*sizeof(u32);
	p347_fpga_info.transfer.cs_change = 1;
	p347_fpga_info.transfer.bits_per_word = 32;
	p347_fpga_info.transfer.speed_hz = p347_fpga_info.new_hz;
	p347_fpga_info.transfer.tx_buf = &spi_out[0];
	p347_fpga_info.transfer.rx_buf = &spi_in[0];

	spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);

	ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
        
        _PDBA("spi receive[%d] = 0x%08x\n",i,spi_in[1]);
    }

/*
    unsigned long spi_out[SPI_TRANSFER_LEN];
    unsigned long spi_in[SPI_TRANSFER_LEN];
    
    for (i=0; i<(SPI_TRANSFER_LEN); i++)
	spi_out[i] = (0x5 << 24) | (0x0b << 16) | i;
    spi_out[SPI_TRANSFER_LEN-1] &= 0xFFFF0000;
    memset(&spi_in[0],0,SPI_TRANSFER_LEN*4);
    
    spi_message_init(&p347_fpga_info.msg);
    //p347_fpga_info.msg.complete = spi_complete_callback;
    //p347_fpga_info.msg.context = NULL;

    p347_fpga_info.transfer.len = SPI_TRANSFER_LEN*sizeof(u32);
    p347_fpga_info.transfer.cs_change = 1;
    p347_fpga_info.transfer.bits_per_word = 32;
    p347_fpga_info.transfer.speed_hz = p347_fpga_info.new_hz;
    p347_fpga_info.transfer.tx_buf = spi_out;
    p347_fpga_info.transfer.rx_buf = spi_in;

    spi_message_add_tail(&p347_fpga_info.transfer,&p347_fpga_info.msg);

    ret = spi_sync(p347_fpga_info.spidev,&p347_fpga_info.msg);
    _PDBA("spi_sync in delayed_function ret=%d\n",ret);
    //_PDBA("spi_async in tasklet ret=%d\n",ret);

    for (i=1; i<SPI_TRANSFER_LEN; i++) {
	_PDBA("spi receive[%d] = 0x%08x\n",i,spi_in[i]);
    }
     */
    gpio_set_value(p347_GPIO_OSCIRQ,1);

}
#endif

//TODO: somewhere should be adc_idx 1 -> 0x1, 2 -> 0x2, 3 -> 0x4, 4 -> 0x8
int spi_write_adc_register(unsigned char adc_idx, unsigned char reg_idx, unsigned short reg_val)
{
    int ret = 0;
#ifdef VERIFY_REGISTERS
    unsigned short check_val = 0;
#endif
    //request register writing
    p347_fpga_info.spi_tx_buff[0] = (1<<28) | (adc_idx<<24) | (reg_idx<<16) | (reg_val);
    p347_fpga_info.spi_tx_buff[1] = 0; //wait
    //_PDBA("spi_write_adc_register %d ch%d with value %d\n",reg_idx,adc_idx,reg_val);
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

#ifdef VERIFY_REGISTERS
    ret = spi_read_adc_register(adc_idx, reg_idx, &check_val);
    if (ret != 0) return ret;
    if (check_val != reg_val)
	_PDBA("CAUTION! adc reg %d write value=%04x, read value=%04x\n",reg_idx,reg_val,check_val);
#endif

    return ret;
}

int spi_write_adc_coefs(unsigned char adc_idx, unsigned char flen, int coefs_number, unsigned long* coefs)
{
    int i;
    int ret;
    unsigned short* codata;

    _PDBA("CONTROL1 for CH%d old = %04x\n",adc_idx,tpi.ABUF[adc_idx-1].par.control1);
    tpi.ABUF[adc_idx-1].par.control1 = (tpi.ABUF[adc_idx-1].par.control1 & 0x00FF) | 0x8000 | ((flen << 5) & 0x01E0);
    TRY_SET(p347_ADC_REG_CONTROL_1, tpi.ABUF[adc_idx-1].par.control1);
    _PDBA("CONTROL1 for CH%d now = %04x\n",adc_idx,tpi.ABUF[adc_idx-1].par.control1);

    codata = kcalloc(coefs_number*2,sizeof(u16),GFP_KERNEL);
    ret = copy_from_user(&codata[0],coefs,coefs_number*4);
    if (ret>0) {
	_PDBA("Copying coefs for channel %d failed\n",adc_idx);
	kfree(codata);
	return -p347_ERROR_ADC_LOAD_COEFS;
    }

    for (i=0; i<coefs_number; i++) {
	msleep(tpi.delays.adc_set_params3);
	ret = spi_write_adc_register(adc_idx,p347_ADC_REG_COEFF_LOW,codata[2*i]);
	if (ret != 0) break;
        msleep(tpi.delays.adc_set_params3);
        ret = spi_write_adc_register(adc_idx,p347_ADC_REG_COEFF_HIGH,codata[2*i+1]);
	if (ret != 0) break;
    }
    
    kfree(codata);
    _PDBA("copied %d coefs for ch %d\n",i,adc_idx);

    return ret;
}

int spi_write_fpga_register(unsigned char reg_idx, unsigned short reg_val)
{
    int ret = 0;
#ifdef VERIFY_REGISTERS
    unsigned short check_val = 0;
#endif
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

#ifdef VERIFY_REGISTERS
    ret = spi_read_fpga_register(reg_idx, &check_val);
    if (ret != 0) return ret;
    if (check_val != reg_val)
	_PDBA("CAUTION! fpga reg %d write value=%04x, read value=%04x\n",reg_idx,reg_val,check_val);
#endif

    return 0;
}

int rot_set_params(t_rot_params* trp) {
    int ret = 0;
    unsigned short val;

    val = trp->pulse_width_min & 0x0000FFFF;
    //_PDBA("Write DO_WIDTH_MIN_LOW value=0x%04x\n",val);
    TRY_SET_FPGA(p347_FPGA_REG_DO_WIDTH_MIN_LOW,val);

    val = (trp->pulse_width_min >>16) & 0x0000FFFF;
    //_PDBA("Write DO_WIDTH_MIN_HIGH value=0x%04x\n",val);
    TRY_SET_FPGA(p347_FPGA_REG_DO_WIDTH_MIN_HIGH,val);

    val = trp->period_min & 0x0000FFFF;
    //_PDBA("Write DO_PERIOD_MIN_LOW value=0x%04x\n",val);
    TRY_SET_FPGA(p347_FPGA_REG_DO_PERIOD_MIN_LOW,val);

    val = (trp->period_min >>16) & 0x0000FFFF;
    //_PDBA("Write DO_PERIOD_MIN_HIGH value=0x%04x\n",val);
    TRY_SET_FPGA(p347_FPGA_REG_DO_PERIOD_MIN_HIGH,val);

    val = trp->period_max & 0x0000FFFF;
    //_PDBA("Write DO_PERIOD_MAX_LOW value=0x%04x\n",val);
    TRY_SET_FPGA(p347_FPGA_REG_DO_PERIOD_MAX_LOW,val);

    val = (trp->period_max >>16) & 0x0000FFFF;
    //_PDBA("Write DO_PERIOD_MAX_HIGH value=0x%04x\n",val);
    TRY_SET_FPGA(p347_FPGA_REG_DO_PERIOD_MAX_HIGH,val);
    
    return ret;
}

int rot_run(unsigned char rot_idx)
{
    int ret = 0;
    if ((rot_idx < 1) || (rot_idx > p347_ROT_CHANNELS_CNT) ) return -EINVAL;

    //power on
    tpi.fparam.settings |= (0x10 << (rot_idx-1));
    //_PDBA("FPGA_SETTINGS write value=0x%08x\n",tpi.fparam.settings);
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    msleep(tpi.delays.rot_run);

    //start
    tpi.fparam.settings |= (0x100 << (rot_idx-1));
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    if (ret == 0) {
        atomic_set(&(tpi.is_rot_running[rot_idx-1]),1);
        _PDBA("ROT channel %d run\n",rot_idx);
    }

    return ret;
}

int rot_stop(unsigned char rot_idx)
{
    int ret = 0;
    if ((rot_idx < 1) || (rot_idx > p347_ROT_CHANNELS_CNT)) return -EINVAL;
    
    tpi.fparam.settings &= ~(0x10 << (rot_idx-1));
    tpi.fparam.settings &= ~(0x100 << (rot_idx-1));
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    if (ret == 0) {
        atomic_set(&(tpi.is_rot_running[rot_idx-1]),0);
        _PDBA("ROT%d stop\n",rot_idx);
    }

    return 0;
}

int warm_up_channel(unsigned char adc_idx, unsigned short settings) {
    int ret = 0;
    if (atomic_read(&(tpi.ABUF[adc_idx-1].is_running))) return -EBUSY;

    tpi.fparam.settings |= (0x1 << (adc_idx-1));
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS, tpi.fparam.settings);
    msleep(tpi.delays.adc_set_params1);

    TRY_SET(p347_ADC_REG_CH_SETTINGS, settings);
    msleep(tpi.delays.adc_set_params3);

    return ret;
}

int warm_off_channel(unsigned char adc_idx) {
    int ret = 0;
    if (atomic_read(&(tpi.ABUF[adc_idx-1].is_running))) return -EBUSY;

    tpi.fparam.settings &= ~(0x1 << (adc_idx-1)); //switch off ON_CHx
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS, tpi.fparam.settings);

    return ret;
}

#define MAX_CRN		5
int adc_set_params(unsigned char adc_idx, unsigned char rot_idx, t_adc_params *par, int buf_len, int usr_len)
{
    int ret;
    int try = 0;
    unsigned short reg_val;
    int done = 0;
    
    if ((!par) || (adc_idx < 1) || (adc_idx > p347_ADC_CHANNELS_CNT)) return -EINVAL;
    if (atomic_read(&(tpi.ABUF[adc_idx-1].is_running))) return -EBUSY;
    
    //power on channel
    tpi.fparam.settings |= (0x1 << (adc_idx-1));
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);
    msleep(tpi.delays.adc_set_params1);

    //set rot channel
    _PDBA("prev value =0x%08x, adc_idx=%d, rot_idx=%d\n",tpi.fparam.rot_ch_mapping,adc_idx,rot_idx);
    tpi.fparam.rot_ch_mapping &= ~(0xFFFF << 4*(adc_idx-1));
    tpi.fparam.rot_ch_mapping |= (rot_idx << 4*(adc_idx-1));
    _PDBA("tpi.fparam.rot_ch_mapping =0x%08x\n",tpi.fparam.rot_ch_mapping);
    TRY_SET_FPGA(p347_FPGA_REG_ROT_CH_MAPPING,tpi.fparam.rot_ch_mapping);
    
    //copy to inner structure
    memcpy(&(tpi.ABUF[adc_idx-1].par),par,sizeof(t_adc_params));
    tpi.ABUF[adc_idx-1].need_len = buf_len*2;
    tpi.ABUF[adc_idx-1].usr_len = usr_len*2;
    //validate
	do {
	    if (try >= MAX_CRN) {
	        //_PDB("Retry limit exceeded, channel broken or not connected\n");
	        tpi.fparam.settings &= ~(0x1 << (adc_idx-1)); //switch off ON_CHx
		if ((tpi.fparam.settings & 0x000F) == 0) tpi.fparam.settings &= 0x7FFF; //switch off BRAM
		TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);
		
		return -p347_ERROR_ADC_CHANNEL_BROKEN;
	    }
	    //msleep(tpi.delays.adc_set_params3);
	    //TRY_SET(p347_ADC_REG_CH_SETTINGS,	par->ch_settings);
	    //msleep(tpi.delays.adc_set_params3);
	    TRY_SET(p347_ADC_REG_CONTROL_2,	par->control2);
	    msleep(tpi.delays.adc_set_params3);
	    TRY_SET(p347_ADC_REG_CONTROL_1,	par->control1);
	    msleep(tpi.delays.adc_set_params3);
	    //test
	    //TRY_SET(p347_ADC_REG_CONTROL_2,	par->control2);
	    //msleep(tpi.delays.adc_set_params3);
	    
	    TRY_SET(p347_ADC_REG_OVERRANGE,	par->overrange);
	    //msleep(1);
	    //TRY_SET(p347_ADC_REG_FILT_COEFF,	par->filt_coeff);
	    msleep(tpi.delays.adc_set_params3);
	    TRY_SET(p347_ADC_REG_GAIN,		par->gain);
	    msleep(tpi.delays.adc_set_params3);
	    TRY_SET(p347_ADC_REG_OFFSET,	par->offset);
	    msleep(tpi.delays.adc_set_params3);
	    //TRY_SET(p347_ADC_REG_CONTROL_1,	par->control1);
	    //msleep(tpi.delays.adc_set_params3);
	    TRY_SET(p347_ADC_REG_CH_SETTINGS,	par->ch_settings);
	    msleep(tpi.delays.adc_set_params3);
	    try += 1;
	    
	    ret = spi_read_fpga_register(p347_FPGA_REG_STATUS,&reg_val);
	    if (!ret) {
		    if (reg_val & (1 << (adc_idx - 1))) {
		        //_PDBA("Channel %d confuguration OK\n",adc_idx);
		        done = 1;
		    } else {
			msleep(tpi.delays.adc_set_params2);
		        _PDBA("Failed configuration of ch %d, STATUS_REG=0x%4x, trying again...\n",adc_idx,reg_val);
		    }
	    } else { //read fail
	        //_PDB("Failed reading status register\n");
	        return ret;
	    }
	    
        //TRY_SET(p347_ADC_REG_CONTROL_2,	ADC_POWER_OFF);

	} while (!done);
    
    return 0;
}

int adc_run(unsigned char adc_idx)
{
    int ret = 0;
    int i = 0;
    if ((adc_idx < 1) || (adc_idx > p347_ADC_CHANNELS_CNT)) return -EINVAL;
    if IS_CH_RUNNING(adc_idx-1) return -EBUSY;

    for (i=0; i<p347_ADC_CHANNELS_CNT; i++)
	if (IS_CH_RUNNING(i)) {
	    ret = 1;
	    break;
	}
    if (ret == 0) //nothing running => reset dma_delayed counter
	atomic_set(&tpi.dma_delayed,0);
    ret = 0;

    //fix dma
    p347_fpga_info.dchain[adc_idx-1].dst_start = tpi.ABUF[adc_idx-1].dma_base;
    omap_set_dma_params(p347_fpga_info.chain_channel[adc_idx-1],&p347_fpga_info.dchain[adc_idx-1]);
    //clean buffer
    
    //memset(tpi.ABUF[adc_idx-1].ptr,0xBB,p347_PUMPBUF_BYTESIZE);
    tpi.ABUF[adc_idx-1].dma_cur = tpi.ABUF[adc_idx-1].dma_base;

    //logic flag should be before real switching,
    //bacause first data may come before flag sets
    atomic_set(&(tpi.ABUF[adc_idx-1].is_running),1);
    atomic_set(&(tpi.ABUF[adc_idx-1].write_pos),0);
    atomic_set(&(tpi.ABUF[adc_idx-1].read_pos),0);
    atomic_set(&(tpi.ABUF[adc_idx-1].proc_pos),0);
    atomic_set(&(tpi.ABUF[adc_idx-1].cur_len),0);
    atomic_set(&(tpi.ABUF[adc_idx-1].proc_len),0);
    
    //TRY_SET(p347_ADC_REG_CONTROL_2,	ADC_POWER_ON);
    //msleep(tpi.delays.adc_run);
    
    //switch on bit in FPGA
    tpi.fparam.settings |= 0x8000;
    //power on channel
    tpi.fparam.settings |= (0x1 << (adc_idx-1));
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    ret = spi_read_fpga32(p347_FPGA_REG_TIMESTAMP1+adc_idx-1,&(tpi.ABUF[adc_idx-1].first_sample_timestamp));
    if (ret != 0) return ret;

    _PDBA("ADC %d run with time offset %ld\n",adc_idx,tpi.ABUF[adc_idx-1].first_sample_timestamp);
    //_PDBA("ADC %d single run\n",adc_idx);
    return 0;
}

int adc_run_sync(t_synctask_channels* tsc)
{
    int ret = 0;
    int i = 0;
    unsigned char adc_idx;
    unsigned char ch_cnt = tsc->adc_ch_cnt;
    unsigned char* _adc_idx = tsc->adc_ch_idx;
    
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++)
	if (IS_CH_RUNNING(i)) {
	    ret = 1;
	    break;
	}
    if (ret == 0) {//nothing running => reset dma_delayed counter
	atomic_set(&tpi.dma_delayed,0);
	atomic_set(&tpi.dma_used,0);
	gpio_set_value(p347_GPIO_OSCIRQ,1);
    }
    ret = 0;

    if ((ch_cnt < 1) || (ch_cnt > p347_ADC_CHANNELS_CNT) || (_adc_idx == NULL)) {
	    _PDB("invalid channel numbers\n");
	return -EINVAL;
    }

    for (ret=0; ret<ch_cnt; ret++) if IS_CH_RUNNING(_adc_idx[ret]-1) {
	    _PDB("Adc channel busy\n");
	    return -EBUSY;
    }

    for (i=0; i<ch_cnt; i++) {
	adc_idx = _adc_idx[i];
	//fix dma
	p347_fpga_info.dchain[adc_idx-1].dst_start = tpi.ABUF[adc_idx-1].dma_base;
	omap_set_dma_params(p347_fpga_info.chain_channel[adc_idx-1],&p347_fpga_info.dchain[adc_idx-1]);
	
        //TRY_SET(p347_ADC_REG_CONTROL_2,	ADC_POWER_ON);

	//memset(tpi.ABUF[adc_idx-1].ptr,0xAA,p347_PUMPBUF_BYTESIZE);
	//invalidate_kernel_vmap_range(tpi.ABUF[adc_idx-1].ptr,0xA00000);
	tpi.ABUF[adc_idx-1].dma_cur = tpi.ABUF[adc_idx-1].dma_base;
	
	atomic_set(&(tpi.ABUF[adc_idx-1].write_pos),0);
	atomic_set(&(tpi.ABUF[adc_idx-1].read_pos),0);
	atomic_set(&(tpi.ABUF[adc_idx-1].proc_pos),0);
	atomic_set(&(tpi.ABUF[adc_idx-1].cur_len),0);
	atomic_set(&(tpi.ABUF[adc_idx-1].proc_len),0);
	atomic_set(&(tpi.ABUF[adc_idx-1].is_running),1);
    	tpi.fparam.settings |= (0x1 << (adc_idx-1));
    }

    TRY_SET_FPGA(p347_FPGA_REG_SYNC,tsc->sync_reg);

    //stabilization delay
    _PDB("Pausing for filter stabilization time...\n");
    msleep(tpi.delays.adc_run_sync);

    //msleep(1000);

    tpi.fparam.settings |= 0x8000;
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    /*
    //read offsets
    for (i=0; i<ch_cnt; i++) {
	    adc_idx = _adc_idx[i] - 1;
        printk("Ch%d: ",_adc_idx[i]);
        ret = spi_read_fpga32(p347_FPGA_REG_TIMESTAMP1+adc_idx,&(tpi.ABUF[adc_idx].first_sample_timestamp));
	    if (ret != 0) return ret;
	    tsc->adc_ch_offset[i] = tpi.ABUF[adc_idx].first_sample_timestamp;
    }
    
    _PDB("Sync run ADC channels:\n");
    for (i=0; i<ch_cnt; i++) {
	    _PDBA("#%d, offset %ld\n",_adc_idx[i],tpi.ABUF[_adc_idx[i]-1].first_sample_timestamp);
    }
    */

    return 0;
}

//int adc_stop_sync(unsigned char ch_cnt, unsigned char* adc_idx, unsigned char rot_idx) 
int adc_stop_sync(t_synctask_channels* tsc)
{
    unsigned char adc_idx;
    unsigned short reg_val;
    int ret;
    int i;

    unsigned char ch_cnt = tsc->adc_ch_cnt;
    unsigned char* _adc_idx = tsc->adc_ch_idx;

    if ((ch_cnt < 1) || (ch_cnt > p347_ADC_CHANNELS_CNT) || (_adc_idx == NULL)) return -EINVAL;
    
    _PDB("ADC ch ");
    for (i=0; i<ch_cnt; i++) {
	    adc_idx = _adc_idx[i];
	    _PDBA(" %d",adc_idx);
	    tpi.fparam.settings &= ~(0x1 << (adc_idx-1));
        TRY_SET(p347_ADC_REG_CONTROL_2,	ADC_POWER_OFF);    
	    atomic_set(&(tpi.ABUF[adc_idx-1].is_running),0);
    }
    _PDB(" sync stop\n");
    //do not stop rot itself
    
    if ((tpi.fparam.settings & 0x000F) == 0) tpi.fparam.settings &= 0x7FFF; //switch off BRAM
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    /*
    //check for fpga overflow counter
    ret = spi_read_fpga_register(p347_FPGA_REG_STATUS,&reg_val);
    if (!ret) {
	_PDBA("FPGA_REG_STATUS = 0x%04x\n",reg_val);
    } else {
	_PDBA("Cannot get FPGA_REG_STATUS with err = %d\n",ret);
    }
    */
    
    return 0;
}
/*
int adc_run_all(void)
{   //TODO: add running check or remove
    int i,ret;

    for (i=0; i<p347_ADC_CHANNELS_CNT; i++)
	atomic_set(&(tpi.ABUF[i].is_running),1);

    tpi.fparam.settings |= 0x80F0;
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    return 0;
}
*/
int adc_stop(unsigned char adc_idx)
{
    int ret;
    if (adc_idx < 1) return -EINVAL;
    
    _PDBA("ADC%d stop\n",adc_idx);

    //switch off channel power supply
    TRY_SET(p347_ADC_REG_CONTROL_2,	ADC_POWER_OFF);
    tpi.fparam.settings &= ~(0x1 << (adc_idx-1)); //switch off ON_CHx
    if ((tpi.fparam.settings & 0x000F) == 0) tpi.fparam.settings &= 0x7FFF; //switch off BRAM
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);
    atomic_set(&(tpi.ABUF[adc_idx-1].is_running),0);

    return 0;
}
/*
int adc_stop_all(void)
{
    int i,ret;
    //TODO: add running check or remove
    //TODO: maybe set some control params
    //switch off all channels
    tpi.fparam.settings &= 0x7F0F; //clear BRAM and all ON_CHx
    TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);

    for (i=0; i<p347_ADC_CHANNELS_CNT; i++)
	atomic_set(&(tpi.ABUF[i].is_running),0);

    return 0;
}
*/
//==========================================================================
//==========================================================================
//==========================================================================

static    t_gpmc_data*			setup_g;
static    t_spi_data*			setup_s;
static    t_ch_params*			ch_p;
static    t_transfer_params*		tp;
static    t_data_buffer*		dbuf;
static    t_rot_params*			trp;
static    t_selfgeneration*		sg;
static    t_synctask_channels*		tsc;
static    t_signal_load*		sl;
static    t_adc_start_single*		tass;
static    t_driver_timings*		tdt;
static    t_adc_coefs*			tac;
static    t_version*			tver;

static char kdata[20];

static unsigned long curlens[p347_ADC_CHANNELS_CNT+1];
static unsigned long addresses[p347_ADC_CHANNELS_CNT];
static unsigned long rot_data[p347_ROT_CHANNELS_CNT];
static unsigned long t_offsets[p347_ADC_CHANNELS_CNT];

//static int p347_fpga_ioctl(struct inode *inode, struct file *filp, uint cmd, unsigned long arg)
static long p347_fpga_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
/*
    t_gpmc_data*		setup_g;
    t_spi_data*			setup_s;
    t_ch_params*		ch_p;
    t_transfer_params*		tp;
    t_data_buffer*		dbuf;
    t_rot_params*		trp;
    t_selfgeneration*		sg;
    t_synctask_channels*	tsc;
    t_signal_load*		sl;
    t_adc_start_single*		tass;
    t_driver_timings*		tdt;
    t_adc_coefs*		tac;
*/
    long ret_code = 0;
    //char kdata[20];
    int i,wcnt,ret,len,p1,p2,tmps = 0;
    unsigned short* us = (unsigned short*)&kdata[0];
    unsigned short reg_idx = 0;
    unsigned short reg_val = 0;
    unsigned short adc_idx;
    
    unsigned long flags = 0;
    unsigned long tmpl = 0;
    /*
    unsigned long curlens[p347_ADC_CHANNELS_CNT+1];
    unsigned long addresses[p347_ADC_CHANNELS_CNT];
    unsigned long rot_data[p347_ROT_CHANNELS_CNT];
    */
    int f_init,f_done = 0;
    
    //unsigned short* tptr;
    //unsigned char testdata[10] = {1,2,3,4,5,6,7,8,9,0};
    //siginfo_t usig;
    
    tpi.deb_irq_print = 0;
    
    if ((p347_fpga_info.user_compatibility == COMPATIBILITY_NOT_CHECKED) && (cmd != p347_IOCTL_CLIENT_CHECK_VERSION))
	return -p347_ERROR_VERSION_NOT_CHECKED;
    if (p347_fpga_info.user_compatibility == COMPATIBILITY_FALSE)
	return -p347_ERROR_VERSION_INCOMPATIBLE;
    
    //lock_kernel();
    //_PDBA("p347_fpga_ioctl start: filp=0x%p, cmd=%d, arg=0x%p\n",(void*)filp,cmd,(void*)arg);
    switch (cmd) {
	//----------------------------------------------------------------------------------------------GPMC
	case p347_IOCTL_WRITE: { //test write
	    if (arg != 0) {
		memset(&kdata[0],0,20);
#ifdef CS67_MODE
		_PDB("CS67_MODE active, so gpmc writing is not supported\n");
#else
		setup_g = (t_gpmc_data*)arg;
		ret = copy_from_user(&kdata[0],setup_g->data,setup_g->word_cnt*sizeof(u16));
		if (ret > 0) _PDBA("p347_IOCTL_WRITE: cannot copy %d bytes from userspace\n",ret);
		for (wcnt=0; wcnt<setup_g->word_cnt; wcnt++)
		    __raw_writew(&kdata[wcnt*2],fpga_mem_write_addr);
#endif
	    }
	break; }
	case p347_IOCTL_READ: {
	    if (arg != 0) {
		memset(&kdata[0],0,20);
		setup_g = (t_gpmc_data*)arg;
		*us = __raw_readw(fpga_mem_read_addr);
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
		ret_code = -p347_ERROR_SPI_INVAL_ARG;
		break;
	    }
	    setup_s = (t_spi_data*)arg;
	    reg_idx = ((unsigned short)(setup_s->data_tx[0]));
	    adc_idx = ((unsigned short)(setup_s->data_tx[1]));
            if ((reg_idx < 1) || (reg_idx > p347_MAX_REG_NUM)) {
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
	    _PDBA("rec[0]=0x%08x, rec[1]=0x%08x, reg_val=0x%04x\n",p347_fpga_info.spi_rx_buff[0],p347_fpga_info.spi_rx_buff[1],reg_val);

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
            if ((reg_idx < 1) || (reg_idx > p347_MAX_REG_NUM)) {
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
	//----------------------------------------------------------------------------------------------Client
    case p347_IOCTL_CLIENT_CHECK_FPGA_STATUS: {
	if (arg != 0) {
	    ret_code = spi_read_fpga_register(p347_FPGA_REG_STATUS, &reg_val);
	    if (ret_code == 0) {
		copy_to_user((unsigned short*)arg,&reg_val,2);
	    }
	} else {
	    ret_code = -p347_ERROR_SYSTEM_INVAL_SETUP;
	}
    break; }
    case p347_IOCTL_CLIENT_SET_DRIVER_DELAYS: {
        if (arg != 0) {
            tdt = (t_driver_timings*)arg;
            ret_code = copy_from_user(&tpi.delays,tdt,sizeof(t_driver_timings));
            _PDBA("New delays for driver: %d,%d,%d,%d,%d,%d msec\n",tpi.delays.rot_run,
                    tpi.delays.adc_set_params1,tpi.delays.adc_set_params2,tpi.delays.adc_set_params3,
                    tpi.delays.adc_run,tpi.delays.adc_run_sync);
        } else {
		ret_code = -p347_ERROR_SYSTEM_INVAL_SETUP;
	}
	//+ add set data pack size
	tmps = (DATA_FRAME_SIZE / 128) - 1;
	_PDBA("DATA_FRAME_SIZE = %d, write %d to FPGA_SETTINGS\n",DATA_FRAME_SIZE,tmps);
	tpi.fparam.settings = ((tmps & 0x7) << 12);
	TRY_SET_FPGA(p347_FPGA_REG_SETTINGS,tpi.fparam.settings);
    break; }
	case p347_IOCTL_CLIENT_SETUP_CHANNEL: {
	    if (arg != 0) {
		    ch_p = (t_ch_params*)arg;
                if (ch_p->idx == 0) {
            	    //_PDBA("setup rot chanel len = %d\n",ch_p->len);
            	    ret_code = 0;
                    //rot_set_params(ch_p->len);
                } else {
            	    //_PDBA("setup channel %d, len = %d data records\n",ch_p->idx,ch_p->len);
		            ret_code = adc_set_params(ch_p->idx,ch_p->rot_idx,&(ch_p->apar),ch_p->len,ch_p->usr_len);
		        }
	    } else {
		    ret_code = -p347_ERROR_SYSTEM_INVAL_SETUP;
	    }
	break; }
	case p347_IOCTL_CLIENT_LOAD_SIGNAL: {
	    if (arg != 0) {
		    sl = (t_signal_load*)arg;
		    if ((sl->u_ptr == NULL) || (sl->len < 1)) {
	            ret_code = -EINVAL;
	            break;
		    }
		    ret_code = load_signal_to_fpga(sl->len,sl->u_ptr);
		    printk("Signal load to FPGA rets %ld",ret_code);
	    } else {
		    ret_code = -EINVAL;
	    }
	break; }
	case p347_IOCTL_CLIENT_COPY_ADC: {
	    if (arg != 0) {
		tp = (t_transfer_params*)arg;
		if (tp->u_ptr == NULL) {
		    ret_code = -EINVAL;
		    break;
		}
		//ch_idx comes from user in 1..N mode, not 0..N-1
		if ((tp->ch_idx < 1) || (tp->ch_idx > p347_ADC_CHANNELS_CNT)) {
		    ret_code = -EINVAL;
		    break;
		}

		//_PDBA("PERFORM_TRANSFER CALL for ch%d\n",tp->ch_idx);
		dbuf = &(tpi.ABUF[tp->ch_idx-1]);

		/*
		if ((tp->fstamp == 0) && (dbuf->first_sample_timestamp != 0)) {
		    put_user(dbuf->first_sample_timestamp,(unsigned long*)(arg+OFFSET_OF(fstamp,t_transfer_params)));
		}
		*/
		//check is there really data for transfer
		if (atomic_read(&(dbuf->cur_len)) >= dbuf->need_len) {
		    //_PDB("Copying data to user!\n");
		    //copy, but check first for length overlap
		    wcnt = atomic_read(&(dbuf->read_pos));
		    len = wcnt + dbuf->need_len;
		    //_PDBA("ch=%d read_pos=%d, need=%d, readend_pos=%d \n",tp->ch_idx,wcnt,dbuf->need_len,len);
		    
		    //!!!spin_lock_irqsave(&(dbuf->lock),flags);
		    
		    if (len <= p347_PUMPBUF_FULLSIZE) { //copy full buffer at once
			
			//tptr = dbuf->ptr+wcnt*sizeof(u16);
			tptr = dbuf->ptr+wcnt;
			
			//_PDBA("COPYING %d bytes with pointer %p for channel %d\n",dbuf->need_len*sizeof(u16),tptr,tp->ch_idx);
			
			/*
			for (i=0; i<dbuf->need_len; i++) {
			    //_PDBA("CHANNEL CONTAINS: %04x %04x\n",tptr[2*i],tptr[2*i+1]);
			    if ((tptr[2*i] == 0) || (tptr[2*i+1] == 0)) {
				if (printk_ratelimit()) _PDB("zeroes in channel\n");
			    }
			}
			*/
			
			//ret = copy_to_user(tp->u_ptr,dbuf->ptr+wcnt*sizeof(u16),dbuf->need_len*sizeof(u16));
			ret = copy_to_user(tp->u_ptr,(dbuf->ptr+wcnt),dbuf->need_len*sizeof(u16));
			if (ret != 0) {
			    _PDBA("%d bytes not copied to user!!!!\n",ret);
			}
			
			wcnt += dbuf->need_len;
			if (wcnt >= p347_PUMPBUF_FULLSIZE) wcnt = 0;
			atomic_set(&(dbuf->read_pos),wcnt);
			//_PDBA("ch %d new rpos = %d\n",tp->ch_idx,wcnt);
		    } else { //copy part from rpos to end and from start to need
			p1 = p347_PUMPBUF_FULLSIZE-wcnt;
			//ret = copy_to_user(tp->u_ptr,dbuf->ptr+wcnt*sizeof(u16),p1*sizeof(u16));//first part
			ret = copy_to_user(tp->u_ptr,(dbuf->ptr+wcnt),p1*sizeof(u16));//first part
			if (ret != 0) {
			    _PDBA("%d bytes not copied to user!!!!\n",ret);
			}
			p2 = dbuf->need_len - p1; //what is left after first part
			//ret = copy_to_user(tp->u_ptr+p1*sizeof(u16),dbuf->ptr,p2*sizeof(u16)); //second part
			ret = copy_to_user((tp->u_ptr+p1),dbuf->ptr,p2*sizeof(u16)); //second part
			if (ret != 0) {
			    _PDBA("%d bytes not copied to user!!!!\n",ret);
			}
			atomic_set(&(dbuf->read_pos),p2);
		    }
		    tmps = atomic_read(&(dbuf->cur_len));
		    tmps -= dbuf->need_len;
		    atomic_set(&(dbuf->cur_len),tmps);
		    //!!!spin_unlock_irqrestore(&(dbuf->lock),flags);
		    
		    //_PDBA("Copied to user ch%d, wpos=%d, rpos=%d, curlen=%d\n",tp->ch_idx-1,
		//	atomic_read(&(dbuf->write_pos)),atomic_read(&(dbuf->read_pos)),atomic_read(&(dbuf->cur_len)));
		} else {
		    ret_code = -ENODATA;
		    break; 
		}
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
	case p347_IOCTL_CLIENT_SHIFT_RPOS: { //shift adc buffer rpos by need_len without copying
	    if ((arg != 0) && (arg <= p347_ADC_CHANNELS_CNT)) {//arg = adc channel number
		dbuf = &(tpi.ABUF[arg-1]);
		//printk("COPY SHIFT ch %d cur_len %d by need_len %d\n",arg,atomic_read(&(dbuf->cur_len)),dbuf->need_len);
		if (atomic_read(&(dbuf->cur_len)) >= dbuf->need_len) {
		    wcnt = atomic_read(&(dbuf->read_pos)) + dbuf->need_len;
		    if (wcnt >= p347_PUMPBUF_FULLSIZE) wcnt -= p347_PUMPBUF_FULLSIZE;
		    atomic_set(&(dbuf->read_pos),wcnt);
		
		    tmps = atomic_read(&(dbuf->cur_len));
		    tmps -= dbuf->need_len;
		    atomic_set(&(dbuf->cur_len),tmps);
		} else {
		    _PDBA("Invalid copy pointer shift! Curlen = %d\n",atomic_read(&(dbuf->cur_len)));
		    ret_code = -ENODATA;
		}
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
	case p347_IOCTL_CLIENT_SHIFT_PROCPOS: {
	    if ((arg != 0) && (arg <= p347_ADC_CHANNELS_CNT)) {//arg = adc channel number
		dbuf = &(tpi.ABUF[arg-1]);
		//printk("PROC SHIFT ch %d proc_len %d by usr_len %d\n",arg,atomic_read(&(dbuf->proc_len)),dbuf->usr_len);
		if (atomic_read(&(dbuf->proc_len)) >= dbuf->usr_len) {
		    wcnt = atomic_read(&(dbuf->proc_pos)) + dbuf->usr_len;
		    if (wcnt >= p347_PUMPBUF_FULLSIZE) wcnt -= p347_PUMPBUF_FULLSIZE;
		    atomic_set(&(dbuf->proc_pos),wcnt);
		
		    tmps = atomic_read(&(dbuf->proc_len));
		    tmps -= dbuf->usr_len;
		    atomic_set(&(dbuf->proc_len),tmps);
		} else {
		    _PDBA("Invalid proc pointer shift! Curlen = %d\n",atomic_read(&(dbuf->proc_len)));
		    ret_code = -ENODATA;
		}
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
	case p347_IOCTL_CLIENT_READ_OFFSETS: {
	    if (arg != NULL) {
		for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
    		    ret = spi_read_fpga32(p347_FPGA_REG_TIMESTAMP1+i,&(tpi.ABUF[i].first_sample_timestamp));
		    if (ret != 0) return ret;
		    t_offsets[i] = tpi.ABUF[i].first_sample_timestamp;
		    //printk("OFFSET%d = 0x%08x\n",i,t_offsets[i]);
		}
		
		ret = copy_to_user((void*)arg,&t_offsets[0],p347_ADC_CHANNELS_CNT*sizeof(unsigned long));
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
	case p347_IOCTL_CLIENT_CHECK_DATA: {
	    if (arg != 0) {
		flush_cache_all();
		for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
		    curlens[i] = atomic_read(&(tpi.ABUF[i].cur_len));
		    //printk("ch%d cur_len = %d\n",i,curlens[i]);
            	    if (atomic_read(&(tpi.ABUF[i].overflow)) > 0) ret_code = -p347_ERROR_ADC_BUFFER_OVERFLOW;
        	}
		copy_to_user((void*)arg,&curlens[0],(p347_ADC_CHANNELS_CNT+1)*sizeof(unsigned long));
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
/*
	//send dma physical addresses to user
	case p347_IOCTL_CLIENT_GET_ADDRESSES: {
	    if (arg != 0) {
		for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
		    addresses[i] = (unsigned long)tpi.ABUF[i].dma_base;
		    addresses[i] += 2*atomic_read(&(tpi.ABUF[i].read_pos));
		}
		copy_to_user((void*)arg,&addresses[0],p347_ADC_CHANNELS_CNT*sizeof(unsigned long));
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
	case p347_IOCTL_CLIENT_GET_BASE_ADDRESSES: {
	    if (arg != 0) {
		for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
		    addresses[i] = (unsigned long)tpi.ABUF[i].dma_base;
		}
		copy_to_user((void*)arg,&addresses[0],p347_ADC_CHANNELS_CNT*sizeof(unsigned long));
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
*/
	case p347_IOCTL_CLIENT_START_CHANNEL: {
	    if (arg != 0) {
		    tass = (t_adc_start_single*)arg;
		
		    if ((tass->idx > 0) && (tass->idx <= p347_ADC_CHANNELS_CNT)) {
		        dbuf = &(tpi.ABUF[tass->idx-1]);
	        	    ret_code = adc_run(tass->idx);
	        	    tass->time_offset = dbuf->first_sample_timestamp;
		    } else {
		        ret_code = -EINVAL;
		    }
	    } else
		    ret_code = -EINVAL;
	break; }
	case p347_IOCTL_CLIENT_STOP_CHANNEL: {
	    if ((arg > 0) && (arg <= p347_ADC_CHANNELS_CNT)) {
	        ret_code = adc_stop((unsigned char)arg);
	    } else
		    ret_code = -EINVAL;
	break; }
	case p347_IOCTL_CLIENT_START_ROT: {
        if (arg != 0) {
            trp = (t_rot_params*)arg;
	        if ((trp->rot_idx > 0) && (trp->rot_idx <= p347_ROT_CHANNELS_CNT)) {
	            ret_code = rot_set_params(trp); //idx
                if (ret_code == 0) {
                    ret_code = rot_run(trp->rot_idx);
                } else {
                    _PDBA("Cannot setup rot channel with error=%d\n",ret_code);
                }
	        } else {
		        ret_code = -EINVAL;
            }
        }
	break; }
	case p347_IOCTL_CLIENT_STOP_ROT: {
	    if ((arg > 0) && (arg <= p347_ROT_CHANNELS_CNT)) {
	        ret_code = rot_stop(arg);//idx
	    } else
		    ret_code = -EINVAL;
	break; }
	case p347_IOCTL_CLIENT_START_SYNC: {
	    //_PDB("p347_IOCTL_CLIENT_START_SYNC\n");
	    if (arg != 0) {
		    tsc = (t_synctask_channels*)arg;
		    ret_code = adc_run_sync(tsc);
		    //ret_code = adc_run_sync(tsc->adc_ch_cnt,tsc->adc_ch_idx,tsc->rot_ch_idx);
	    } else
		    ret_code = -EINVAL;
	break; }
	case p347_IOCTL_CLIENT_STOP_SYNC: {
	    if (arg != 0) {
		    tsc = (t_synctask_channels*)arg;
            ret_code = adc_stop_sync(tsc);
		    //ret_code = adc_stop_sync(tsc->adc_ch_cnt,tsc->adc_ch_idx,tsc->rot_ch_idx);
	    } else
		    ret_code = -EINVAL;
	break; }
	case p347_IOCTL_CLIENT_SET_CC: {
    	    _PDB("p347_IOCTL_CLIENT_SET_CC, do nothing\n");
	    //TODO: write calibration coeffs to FPGA (for each ADC chip)
	break; }
	case p347_IOCTL_CLIENT_WARM_UP: {
	    reg_val = (arg >> 16) & 0x0000FFFF;
	    adc_idx = arg & 0x0000FFFF;
	    if ((adc_idx < 1) || (adc_idx > p347_ADC_CHANNELS_CNT) || (reg_val == 0))
		ret_code = -EINVAL;
	    else {
		ret_code = warm_up_channel(adc_idx,reg_val);
	    }
	break; }
	case p347_IOCTL_CLIENT_WARM_OFF: {
	    adc_idx = arg & 0x0000FFFF;
            if ((adc_idx < 1) || (adc_idx > p347_ADC_CHANNELS_CNT))
		ret_code = -EINVAL;
	    else {
		ret_code = warm_off_channel(adc_idx);
	    }
	break; }
	case p347_IOCTL_CLIENT_CHECK_VERSION: {
	    if (arg != 0) {
		tver = (t_version*)arg;
		_PDBA("DRIVER: major=%d, minor=%d, tail=%d\n",VERSION_MAJOR,VERSION_MINOR,VERSION_TAIL);
		_PDBA("USER: major=%ld, minor=%ld, tail=%ld\n",tver->major,tver->minor,tver->tail);
	        if ((tver->major != VERSION_MAJOR) || (tver->minor != VERSION_MINOR)) {
		    p347_fpga_info.user_compatibility = COMPATIBILITY_FALSE;
		    ret_code = -p347_ERROR_VERSION_INCOMPATIBLE;
		} else {
		    p347_fpga_info.user_compatibility = COMPATIBILITY_TRUE;
		}
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
	//------------------------------------------------------------------------------test
	case p347_IOCTL_GET_INT_CNT: {
	    if (arg>0) {
		spin_lock_irqsave(&irq_lock,flags);
		tmpl = sul_icnt;
		spin_unlock_irqrestore(&irq_lock,flags);
		_PDBA("GET_INT_CNT = %d, counter2=%d\n",atomic_read(&p347_fpga_info.i_cnt),sul_icnt);
		//put_user(atomic_read(&p347_fpga_info.i_cnt),(unsigned long*)arg);
		wcnt = atomic_read(&tpi.dma_delayed);
		if (wcnt > 0) _PDBA("GET_INT_CNT DMA delayed counter = %d\n",wcnt);
		//put_user(p347_fpga_info.i_cnt,(unsigned long*)arg);
	    }
	break; }
	case p347_IOCTL_GENERATE_PULSE: {
	    if (arg>0) {
		//usleep_range for useconds
		sg = (t_selfgeneration*)arg;
		if ((sg->count <1) || (sg->us_width<1) || (sg->us_idle<1)) {
		
		}
	    }
	break; }
	case p347_IOCTL_SET_DEBUG_PRINTS: {
	    tpi.deb_irq_print = arg & 0xF;
	    _PDBA("irq debug printing set to %d\n",tpi.deb_irq_print);
	break; }
	case p347_IOCTL_FPGA_RESET: {
	    gpio_set_value(p347_GPIO_FPGA_RESET,0);
	    msleep(1);
	    gpio_set_value(p347_GPIO_FPGA_RESET,1);
	    i = 0;
	    do {
    		f_init = __gpio_get_value(p347_GPIO_FPGA_INIT);
    		f_done = __gpio_get_value(p347_GPIO_FPGA_DONE);
    		if ((f_init != 0) & (f_done != 0)) {
        	    _PDB("FPGA is configured OK\n");
        	break; }
    		if ((f_init == 0) & (f_done != 0)) {
        	    _PDB("FPGA FIRMWARE CRC ERROR\n");
        	    ret_code = -p347_ERROR_FPGA_CRC_FIRMWARE;
    		}
    		msleep(FPGA_LOAD_WAIT_MSEC);
	    } while (i<FPGA_LOAD_WAIT_CYCLES);

	    if (i>=FPGA_LOAD_WAIT_CYCLES) {
    		_PDB("FPGA NOT CONFIGURED\n");
    		if (ret_code != 0)
    		    ret_code = -p347_ERROR_FPGA_NOT_CONFIGURED;
	    }
	break; }
	case p347_IOCTL_CLIENT_LOAD_ADC_COEFS: {
	    if (arg > 0) {
		tac = (t_adc_coefs*)arg;
		ret_code = spi_write_adc_coefs(tac->ch_idx,tac->flen,tac->coefs_number,tac->coefs);
	    } else {
		ret_code = -EINVAL;
	    }
	break; }
    case p347_IOCTL_CLIENT_READ_ROT: {
        if (arg > 0) {
            for (i=0; i<p347_ROT_CHANNELS_CNT; i++)
                if (IS_ROT_RUNNING(i)) {
                    ret_code = spi_read_fpga32(p347_FPGA_REG_DO1+i,&rot_data[i]);
                    if (ret_code != 0) break;
                } else {
                    rot_data[i] = 0;
                }
            if (ret_code == 0) {
                copy_to_user((void*)arg,&rot_data[0],p347_ROT_CHANNELS_CNT*4);
            }
        } else
            ret_code = -EINVAL;
    break; }
	case p347_IOCTL_TEST_ASM: {
	    printk("================================\n");
	    /*
	    msleep(10);
	    asm("push    {r8-r9, lr}");
	    asm("ldr     r8, =0x49020000");
	    asm("mov     r9, #'T'");
	    asm("strb    r9, [r8, #0]");
	    asm("pop     {r8-r9, pc}");
	    msleep(10);
	    */
	    
	    msleep(10);
	    asm("push    {r8-r10, lr}");
	    
	    asm("ldr r8,     =0xfb05003c");
	    asm("ldr r10,    =0x02000000");
	    asm("ldr r9,     [r8]");
	    asm("tst r9,     r10");
	    asm("bne         bit_clear");
	    //asm("teq r9,     r10");
	    //asm("bne         bit_clear");
	    asm("bit_set:    ldr r8, =0xfb050094");
	    asm("str r10,    [r8]");
	    asm("b           bit_end");
	    asm("bit_clear:  ldr r8, =0xfb050090");
	    asm("str r10,    [r8]");
	    asm("bit_end:");
	    
	    asm("pop     {r8-r10, pc}");
	    msleep(10);
	    /*
	    ret = gpio_get_value(p347_GPIO_OSCIRQ);
	    _PDBA("cs6 out = %d\n",ret);
	    if (ret != 0) {
		gpio_set_value(p347_GPIO_OSCIRQ,0);
	    } else {
	        gpio_set_value(p347_GPIO_OSCIRQ,1);
	    }
	    ret = gpio_get_value(p347_GPIO_OSCIRQ);
	    _PDBA("cs6 out = %d\n",ret);
	    */
	break; }
	default: {
	    _PDBA("p347_ioctl unknown cmd=%d\n",cmd);
	break; }
    };
    //_PDB("p347_fpga_ioctl end\n");
    
    //unlock_kernel();
    return ret_code;
}

//==========================================================================
//==========================================================================		COMMON
//==========================================================================

static int p347_fpga_open(struct inode *inode, struct file *filp)
{
    int i;
    _PDBA("p347_fpga_open: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);

    if (p347_fpga_info.opened > 0) {
	    printk("ERROR: device is already opened!\n");
	    return -EBUSY;
    }    

    for (i=0; i<p347_ADC_CHANNELS_CNT; i++)
        atomic_set(&(tpi.ABUF[i].is_running),0);
    for (i=0; i<p347_ROT_CHANNELS_CNT; i++)
        atomic_set(&(tpi.is_rot_running[i]),0);

    p347_fpga_info.opened=1;

    return nonseekable_open(inode,filp);
}

//==========================================================================

static int p347_fpga_release(struct inode *inode, struct file *filp)
{
    int i;
    _PDBA("p347_fpga_release: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);
    p347_fpga_info.opened=0;
    p347_fpga_info.user_compatibility = COMPATIBILITY_NOT_CHECKED;

    for (i=0; i<p347_ROT_CHANNELS_CNT; i++)
        if (IS_ROT_RUNNING(i)) {
	    _PDBA("USER FORGOT TO STOP ROT CHANNEL %d, FORCING STOP NOW\n",i);
	    rot_stop(i+1);
        }
        
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++)
	if (IS_CH_RUNNING(i)) {
	    _PDBA("USER FORGOT TO STOP ADC CHANNEL %d, FORCING STOP NOW\n",i);
	    adc_stop(i+1);
	}

    return 0;
}

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


#define set_read_print(cs,reg,var,name) \
    my_gpmc_cs_write_reg(cs, reg, var);\
    var = my_gpmc_cs_read_reg(cs, reg);\
    _PDBA("CS%d "name" reg value = 0x%x\n",cs,var);

void set_timings(unsigned char gcs) {
    unsigned char div,mult;
    u32 reg_val;

    div = 1;
    mult = 1;                     //22
    //cswrofftime = 5 ticks, csrdofftime = 15 ticks, csextradelay = 0, csontime = 0
    reg_val = (5*mult << 16) | (23*mult << 8) | (0 << 7) | (0*mult);
    set_read_print(gcs,GPMC_CS_CONFIG2, reg_val,"CONFIG2");
    //advwrofftime = 2, advrdofftime = 4, advextradelay = 0, advontime = 1
    reg_val = (2*mult << 16) | (4*mult << 8) | (0*mult << 7) | (1*mult);
    set_read_print(gcs,GPMC_CS_CONFIG3, reg_val,"CONFIG3");
    //weofftime = 5, weextradelay = 0, weontime = 3, oeofftime = 13, oeextradelay = 0, oeontime = 4
    reg_val = (5*mult << 24) | (0*mult << 23) | (3*mult << 16) | (22*mult << 8) | (0 << 7) | (6*mult);
    set_read_print(gcs,GPMC_CS_CONFIG4, reg_val,"CONFIG4");
    //pageburstaccesstime = 1, rdaccesstime = 6, wrcycletime = 7, rdcycletime = 15
    reg_val = (2*mult << 24) | (7*mult << 16) | (7*mult << 8) | (8*mult);//10
    set_read_print(gcs,GPMC_CS_CONFIG5, reg_val,"CONFIG5");
    //wraccesstime = 3, wrdataonadmuxbus = 2, cycle2cycledelay = 1
    //cycle2cyclesamecsen = 0, cycle2cyclediffcsen = 1, busturnaround = 1
    reg_val = my_gpmc_cs_read_reg(gcs, GPMC_CS_CONFIG6);
    reg_val &= 0x80000000; //do not modify bit 31
    
    //reg_val |= (3*mult << 24) | (2*mult << 16) | (0*mult << 8) | (0*mult << 7) | (0*mult << 6) | (0*mult);
    reg_val |= (3*mult << 24) | (2*mult << 16) | (1*mult << 8) | (0*mult << 7) | (1*mult << 6) | (1*mult);
    set_read_print(gcs,GPMC_CS_CONFIG6, reg_val,"CONFIG6");
    //config1
    reg_val = 0x68801000;
    switch (GPMC_MHZ) {
	case 80: {
	    reg_val |= 0x11; //TIMEPARAGRANULARITY and GPMC_CLK = GPMC_FCLK/2
	break; }
	case 166: {
	    //reg_val |= 0x10; //TIMEPARAGRANULARITY and GPMC_CLK = GPMC_FCLK
	break; }
    };

    set_read_print(gcs,GPMC_CS_CONFIG1, reg_val,"CONFIG1");
}

void print_gpmc_padconf(unsigned long base_addr) {
    unsigned long padconf_addr;
    unsigned long reg_val;
    int i;
    unsigned char bit_inen, bit_pullud, bit_pullen = 0;
    
    for (i=0; i<22; i++) {
	padconf_addr = base_addr + 0x207C + i*4;
	reg_val = __raw_readl(padconf_addr);
	printk("REG 0x%p: value 0x%p\n",(void*)padconf_addr,(void*)reg_val);
	if (reg_val & 0x01000000) bit_inen = 1; else bit_inen = 0;
	if (reg_val & 0x00100000) bit_pullud = 1; else bit_pullud = 0;
	if (reg_val & 0x00080000) bit_pullen = 1; else bit_pullen = 0;
	printk("    High word: INPUTENABLE=%d, PULLUP/DOWN=%d, PULLENABLE=%d\n",bit_inen,bit_pullud,bit_pullen);
	if (reg_val & 0x00000100) bit_inen = 1; else bit_inen = 0;
	if (reg_val & 0x00000010) bit_pullud = 1; else bit_pullud = 0;
	if (reg_val & 0x00000008) bit_pullen = 1; else bit_pullen = 0;
	printk("    Low word: INPUTENABLE=%d, PULLUP/DOWN=%d, PULLENABLE=%d\n",bit_inen,bit_pullud,bit_pullen);
    }
}

void print_clken_registers(void) {
    unsigned short mul,div,m2,dl3,dl4,src = 0;
    unsigned long cm_base = (unsigned long)ioremap(0x48000000,SZ_4K);
    unsigned long reg_val = __raw_readl(cm_base+0x4A00);
    printk("CM_FCLKEN1_CORE = 0x%08lx\n",reg_val);
    reg_val = __raw_readl(cm_base+0x4A10);
    printk("CM_ICLKEN1_CORE = 0x%08lx\n",reg_val);
    //--------------------------------------------------
    reg_val = __raw_readl(cm_base+0x4924);
    if (reg_val & 0x1) printk("MPU_CLK is locked to SYS_CLK\n");
    else printk("MPU_CLK is bypassed and = DPLL1_FCLK\n");
    //--
    reg_val = __raw_readl(cm_base+0x4940);
    mul = (reg_val >> 8) & 0x7FF;
    div = reg_val & 0x7F;
    src = (reg_val >> 19) & 0x7;
    printk("CM_CLKSEL1_PLL_MPU = 0x%08lx, mult=%d, div=%d, m2=%d\n",reg_val,mul,div,src);
    reg_val = __raw_readl(cm_base+0x4944);
    div = reg_val & 0x1F;
    printk("CM_CLKSEL2_PLL_MPU = 0x%08lx, div=%d\n",reg_val,div);
    //--------------------------------------------------
    reg_val = __raw_readl(cm_base+0x4d40);
    mul = (reg_val >> 16) & 0x7FF;
    div = (reg_val >> 8) & 0x7F;
    m2 = (reg_val >> 27) & 0x1F;
    printk("CM_CLKSEL1_PLL = 0x%08lx, mult=%d, div=%d, M2=%d\n",reg_val,mul,div,m2);
    reg_val = __raw_readl(cm_base+0x4a40);
    dl3 = (reg_val & 0x3);
    dl4 = (reg_val >> 2) & 0x3;
    printk("CM_CLKSEL_CORE = 0x%08lx, dl3=%d, dl4=%d\n",reg_val,dl3,dl4);
    cm_base = ioremap(0x48300000,SZ_4K);
    reg_val = __raw_readl(cm_base+0x6d40);
    printk("PRM_CLKSEL = 0x%08lx\n",reg_val);
    printk("Bits[2:0] -> 0=12Mhz, 1=13, 2=19.2, 3= 26, 4=38.4, 5=16.8\n");
}

/*
#define psp_padconf(addr,offset,value)\
    padconf_addr = scm_base + addr;\
    reg_val = __raw_readl(padconf_addr);\
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);\
    if (offset) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;\
    reg_val |= (value << offset);\
    __raw_writel(reg_val,padconf_addr);\
    reg_val = __raw_readl(padconf_addr);\
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val)
*/
//less print
#define psp_padconf(addr,offset,value)\
    padconf_addr = scm_base + addr;\
    reg_val = __raw_readl(padconf_addr);\
    if (offset) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;\
    reg_val |= (value << offset);\
    __raw_writel(reg_val,padconf_addr)

#ifdef USE_FIQ
    extern unsigned char p347_fiq_start;
    extern unsigned char p347_fiq_end;
#endif

static int __init p347_fpga_init(void)
{
    int ret,i;
    u32 reg_val,old;
    size_t tmp_sz = 0;
    unsigned long rate;
    unsigned long scm_base = (unsigned long)ioremap(0x48000000,SZ_4K);
    unsigned long cm_addr = scm_base + 0x4d40;
    unsigned long padconf_addr;

    //unsigned long gpio_base = (unsigned long)ioremap(0x48310000,SZ_4K); //GPIO bank 1
    unsigned long gpio_base = (unsigned long)ioremap(0x49054000,SZ_4K); //GPIO bank 4
    unsigned long dmareg_base = (unsigned long)ioremap(0x48056000,SZ_4K);
    unsigned long mpuintc_base = (unsigned long)ioremap(0x48200000,SZ_4K);
    unsigned long tmp_addr;

    ipend0 = mpuintc_base + 0x98;
    ipend1 = ipend0 + 0x20;
    ipend2 = ipend0 + 0x40;

    spin_lock_init(&irq_lock);
    spin_lock_init(&dma_lock);
    
    //tpi.client_pid = -1;
    //TODO: universal free-and-clear function
    
    _PDBA("Init start: %s\n",p347_DRV_VERSION);
    print_clken_registers();
    
    p347_fpga_info.err_cnt = 0;
    
    //ret = gpmc_cs_request(p347_GPMC_FPGA_CS_READ,SZ_16M,&fpga_mem_read_base);
    ret = gpmc_cs_request(p347_GPMC_FPGA_CS_READ,SZ_2K,&fpga_mem_read_base);
    _PDBA("gpmc_cs_request READ ret=%d\n",ret);
    fpga_mem_read_addr = (unsigned long)ioremap(fpga_mem_read_base,SZ_2K);
    _PDBA("read mem base=0x%p, remap=0x%p\n",(void*)fpga_mem_read_base,(void*)fpga_mem_read_addr);

#ifndef CS67_MODE
    //ret = gpmc_cs_request(p347_GPMC_FPGA_CS_WRITE,SZ_16M,&fpga_mem_write_base);
    ret = gpmc_cs_request(p347_GPMC_FPGA_CS_WRITE,SZ_2K,&fpga_mem_write_base);
    _PDBA("gpmc_cs_request WRITE ret=%d\n",ret);
    fpga_mem_write_addr = (unsigned long)ioremap(fpga_mem_write_base,SZ_2K);
    _PDBA("write mem base=0x%p, remap=0x%p\n",(void*)fpga_mem_write_base,(void*)fpga_mem_write_addr);
#endif

    memset(&p347_fpga_info,0,sizeof(p347_fpga_device_t));
    atomic_set(&p347_fpga_info.i_cnt,0);
    //p347_fpga_info.i_cnt = 0;
#ifndef DMA_DIRECT
    //-----------------------------------------------------------memory allocations
    tmp_sz = DATA_FRAME_SIZE*RD_BURST_LEN*2; //dma buffer size
    d_ret = dma_alloc_coherent(NULL,tmp_sz,&p347_fpga_info.dma_rd_addr,GFP_DMA);
    if (d_ret == NULL) {
	_PDBA("Cannot allocate space for gpmc dma rx buffer (%d bytes)\n",tmp_sz);
	return -ENOMEM;
    }
    _PDBA("dma_alloc_coherent ret=0x%p, addr =0x%08x\n",d_ret,p347_fpga_info.dma_rd_addr);
#endif    
    tmp_sz = SPI_RXBUFF_LEN*sizeof(u32);
    p347_fpga_info.spi_rx_buff = kzalloc(tmp_sz,GFP_KERNEL);
    if (p347_fpga_info.spi_rx_buff == NULL) {
	_PDBA("Cannot allocate space for spi_rx_buff (%d bytes)\n",tmp_sz);
	dma_free_coherent(NULL,RD_BURST_LEN*2,d_ret,p347_fpga_info.dma_rd_addr);
	return -ENOMEM;
    }
    _PDBA("spi_rx_buff kzallocated in 0x%p\n",p347_fpga_info.spi_rx_buff);
    
    tmp_sz = SPI_TXBUFF_LEN*sizeof(u32);
    p347_fpga_info.spi_tx_buff = kzalloc(tmp_sz,GFP_KERNEL);
    if (p347_fpga_info.spi_tx_buff == NULL) {
	_PDBA("Cannot allocate space for spi_tx_buff (%d bytes)\n",tmp_sz);
	dma_free_coherent(NULL,RD_BURST_LEN*2,d_ret,p347_fpga_info.dma_rd_addr);
	kfree(p347_fpga_info.spi_rx_buff);
	return -ENOMEM;
    }
    _PDBA("spi_tx_buff kzallocated in 0x%p\n",p347_fpga_info.spi_tx_buff);
    
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
    //-----------------------------------------------------------mcspi3 pad configuration
    psp_padconf(PADCONF_SPI3_SIMO_ADDR,PADCONF_SPI3_SIMO_OFFSET,PADCONF_SPI3_SIMO_VALUE);
    psp_padconf(PADCONF_SPI3_SOMI_ADDR,PADCONF_SPI3_SOMI_OFFSET,PADCONF_SPI3_SOMI_VALUE);
    psp_padconf(PADCONF_SPI3_CS1_ADDR,PADCONF_SPI3_CS1_OFFSET,PADCONF_SPI3_CS1_VALUE);
    psp_padconf(PADCONF_SPI3_CLK_ADDR,PADCONF_SPI3_CLK_OFFSET,PADCONF_SPI3_CLK_VALUE);
    psp_padconf(PADCONF_SPI3_CS0_ADDR,PADCONF_SPI3_CS0_OFFSET,PADCONF_SPI3_CS0_VALUE);
    
    //-----------------------------------------------------------
    
    gpmc_base = ioremap(0x6e000000,SZ_4K);
    _PDBA("ioremap ret=0x%p\n",(void*)gpmc_base);
    
    //-----------------------------------------------------------dpll3 clock divider (make low rate)
  //comment if no need to modify system clock
    if (GPMC_MHZ < 80) {
	reg_val = __raw_readl(cm_addr);
	_PDBA("read addr 0x%p ret 0x%x\n",(void*)cm_addr,reg_val);
	reg_val &= 0x07FFFFFF;
	//reg_val |= (16 << 27); //2.66MHz = 166/4/16 (4 is gpmc_fclk divider)
	//reg_val |= (2<<27);
	if (GPMC_MHZ == 20) reg_val |= (8<<27); //166/8=20.75MHz
	if (GPMC_MHZ == 40) reg_val |= (4<<27); //166/8=41.5MHz
	__raw_writel(reg_val,cm_addr);
	reg_val = __raw_readl(cm_addr);
	_PDBA("read addr 0x%p ret 0x%x\n",(void*)cm_addr,reg_val);
    }
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
	_PDBA("gpmc_fck rate=%ld\n",rate);
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
	_PDBA("dpll3_m2_ck rate=%ld\n",rate);
    }

/*
    //clock enable (iclk) for GPIO1
    reg_val = __raw_readl(scm_base+0x4c10);
    _PDBA("default CM_ICLKEN_WKUP register = 0x%x\n",reg_val);
    reg_val = reg_val | 0x8; //set bit3 (EN_GPIO1) to 1
    __raw_writel(reg_val,scm_base+0x4c10);
    reg_val = __raw_readl(scm_base+0x4c10);
    _PDBA("new CM_ICLKEN_WKUP register = 0x%x\n",reg_val);
    //clock enable (fclk) for GPIO1
    reg_val = __raw_readl(scm_base+0x4c00);
    _PDBA("default CM_FCLKEN_WKUP register = 0x%x\n",reg_val);
    reg_val = reg_val | 0x8; //set bit3 (EN_GPIO1) to 1
    __raw_writel(reg_val,scm_base+0x4c00);
    reg_val = __raw_readl(scm_base+0x4c00);
    _PDBA("new CM_FCLKEN_WKUP register = 0x%x\n",reg_val);
    //print CM_IDLEST_WKUP
    reg_val = __raw_readl(scm_base+0x4c20);
    _PDBA("read-only CM_IDLEST_WKUP register = 0x%x\n",reg_val);
    //autoidle register
    reg_val = __raw_readl(scm_base+0x4c30);
    _PDBA("read-only CM_AUTOIDLE_WKUP register = 0x%x\n",reg_val);
*/

    /*
    //request datacopy gpio
    ret = gpio_request(p347_DATACOPY_GPIO,"gpio_datacopy");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_datacopy, ret=%d\n",ret);
	return ret;
    }
    
    padconf_addr = scm_base + PADCONF_DATACOPY_ADDR;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0xFFFF0000;
    reg_val |= (PADCONF_DATACOPY_VALUE << PADCONF_DATACOPY_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    
    gpio_direction_output(p347_DATACOPY_GPIO,1);
    gpio_set_value(p347_DATACOPY_GPIO,1);
    */
    //request dispatch gpio
    ret = gpio_request(p347_DISPATCH_GPIO,"gpio_dispatch");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_dispatch, ret=%d\n",ret);
	return ret;
    }
    
    padconf_addr = scm_base + PADCONF_DISPATCH_ADDR;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0xFFFF0000;
    reg_val |= (PADCONF_DISPATCH_VALUE << PADCONF_DISPATCH_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    
    gpio_direction_output(p347_DISPATCH_GPIO,1);
    gpio_set_value(p347_DISPATCH_GPIO,1);
    
    //-----------------------------------------------------------request pins
    /*
    _PDB("Changing IRQ priorities:\n");
    for (i=0; i<96; i++) {
	tmp_addr = mpuintc_base + 0x100 + 4*i;
	
	//reg_val = __raw_readl(tmp_addr);
	//old = (reg_val >> 2) & 0x3F;
	
	reg_val = 4;
	if (i != INT_34XX_GPIO_BANK4) __raw_writel(reg_val,tmp_addr);
	
	reg_val = __raw_readl(tmp_addr);
	old = (reg_val >> 2) & 0x3F;
	printk("%d, ",old);
	if ((i==19) || (i==39) || (i==59) || (i==79)) printk("\n");
    }
    */
    
    /*
    //try to set high interrupt priority to GPIO_BANK4
    tmp_addr = ioremap(0x48200000,SZ_4K)+0x100+0x4*INT_34XX_GPIO_BANK4;
    //__raw_writel(0x1,tmp_addr);
    reg_val = __raw_readl(tmp_addr);
    old = (reg_val >> 2) & 0x3F;
    _PDBA("INTCPS_ILR%d = %08x, priority=%d\n",INT_34XX_GPIO_BANK4,reg_val,old);
    */
    
    p347_fpga_info.irqnum = gpio_to_irq(p347_GPIO_FPGA_IRQ);
    _PDBA("gpio_to_irq ret irqnum=%d\n",p347_fpga_info.irqnum);

#ifdef USE_FIQ
    ret = claim_fiq(&fh);
    if (ret) {
	_PDBA("Failed to claim FIQ, err = %d\n",ret);
    } else {
	set_fiq_handler(&fh,&p347_fiq_start - &p347_fiq_end);
	//set_fiq_regs(&regs);
	enable_fiq(p347_GPIO_FPGA_IRQ);
    }
#endif

    ret = gpio_request(p347_GPIO_FPGA_IRQ,"gpio_fpga_irq");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_fpga_irq, ret=%d\n",ret);
	return ret;
    }
    _PDB("gpio_fpga_irq registered\n");
    

    
    ret = request_irq(p347_fpga_info.irqnum,p347_fpga_irq_handler,0,"p347_fpga_irq",&p347_fpga_info);
    _PDBA("request_irq %d ret %d\n",p347_fpga_info.irqnum,ret);

    irqstatus_reg_addr = gpio_base + 0x18;
    gpio_data_reg_addr = gpio_base + 0x38;
    gpio_inen_reg_addr = gpio_base + 0x1C;

    old = __raw_readl(gpio_base + 0x30); //GPIO_CTRL
    reg_val = 0; //enable module, no gating
    __raw_writel(reg_val,gpio_base + 0x30);
    reg_val = __raw_readl(gpio_base + 0x30);
    _PDBA("CTRL for GPIO old=0x%08x, new=0x%08x\n",old,reg_val);

    old = __raw_readl(gpio_base + 0x10); //GPIO_SYSCONFIG
    //reg_val = 0x8; //No-idle mode
    reg_val = 0x19; //Smart autoidle
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
    //_PDB("gpio_oscirq
    //pad configuration for CS6
    padconf_addr = scm_base + PADCONF_OSCIRQ;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0x0000FFFF;
    reg_val |= (PADCONF_OSCIRQ_VALUE << PADCONF_OSCIRQ_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    
    gpio_direction_output(p347_GPIO_OSCIRQ,1);
    gpio_set_value(p347_GPIO_OSCIRQ,1);
#ifndef USE_CS7_AS_CS
    ret = gpio_request(p347_GPIO_OSCDMA,"gpio_oscdma");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_oscdma, ret=%d\n",ret);
	return ret;
    }
    //pad configuration for CS7
    padconf_addr = scm_base + PADCONF_OSCDMA;
    reg_val = __raw_readl(padconf_addr);
    reg_val &= 0xFFFF0000;
    reg_val |= PADCONF_OSCDMA_VALUE;
    __raw_writel(reg_val,padconf_addr);
    
    gpio_direction_output(p347_GPIO_OSCDMA,1);
#endif
    
#endif

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

    //-----------------------------------------------------------fpga_check pad configuration
    padconf_addr = scm_base + PADCONF_FPGA_DONE_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read fpga done padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    if (PADCONF_FPGA_DONE_OFFSET) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;
    reg_val |= (PADCONF_FPGA_DONE_VALUE << PADCONF_FPGA_DONE_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read fpga done padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);

    padconf_addr = scm_base + PADCONF_FPGA_INIT_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read fpga init padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    if (PADCONF_FPGA_INIT_OFFSET) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;
    reg_val |= (PADCONF_FPGA_INIT_VALUE << PADCONF_FPGA_INIT_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read fpga init padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);

    ret = gpio_request(p347_GPIO_FPGA_DONE,"gpio_fpga_done");
    if (ret != 0) {
	    _PDBA("ERROR: cannot request gpio_fpga_done, ret=%d\n",ret);
	    return ret;
    }

    ret = gpio_request(p347_GPIO_FPGA_INIT,"gpio_fpga_init");
    if (ret != 0) {
	    _PDBA("ERROR: cannot request gpio_fpga_init, ret=%d\n",ret);
	    return ret;
    }

    gpio_direction_input(p347_GPIO_FPGA_DONE);
    gpio_direction_input(p347_GPIO_FPGA_INIT);

    //-----------------------------------------------------------gpmc request and setup
    reg_val = __raw_readl(gpmc_base+0x50);//GPMC_CONFIG
    printk("GPMC_CONFIG old value = %04x\n",(reg_val&0xFFFF));
    reg_val |= 2; //limited address support
    __raw_writel(reg_val,gpmc_base+0x50);
    reg_val = __raw_readl(gpmc_base+0x50);//GPMC_CONFIG
    printk("GPMC_CONFIG new value = %04x\n",(reg_val&0xFFFF));
    //set timings
    set_timings(p347_GPMC_FPGA_CS_READ);
    set_timings(p347_GPMC_FPGA_CS_WRITE);

    
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	_create_adc_buffer((t_data_buffer*)(&tpi.ABUF[i]));
#ifdef USE_DMA_CHAIN
	p347_fpga_info.chain_channel[i] = -1;
	if (i == (p347_ADC_CHANNELS_CNT - 1)) {
	    ret = omap_request_dma(0,"p347_dma_chain",p347_fpga_dma_callback,&p347_fpga_info,&p347_fpga_info.chain_channel[i]);
	} else {
	    ret = omap_request_dma(0,"p347_dma_chain",NULL,&p347_fpga_info,&p347_fpga_info.chain_channel[i]);
	}
	_PDBA("omap_request_dma chain %d ret %d, channel = %d\n",i,ret,p347_fpga_info.chain_channel[i]);
	
	p347_fpga_info.dchain[i].data_type = OMAP_DMA_DATA_TYPE_S16;
	p347_fpga_info.dchain[i].src_start = fpga_mem_read_base;
	p347_fpga_info.dchain[i].src_amode = OMAP_DMA_AMODE_POST_INC;
	p347_fpga_info.dchain[i].dst_amode = OMAP_DMA_AMODE_POST_INC;
	p347_fpga_info.dchain[i].dst_ei = 1;
	p347_fpga_info.dchain[i].dst_fi = 1;
	p347_fpga_info.dchain[i].elem_count = 8;
	p347_fpga_info.dchain[i].frame_count = FRAMES_FOR_SINGLE_CHANNEL;
	p347_fpga_info.dchain[i].sync_mode = OMAP_DMA_SYNC_BLOCK;
	
	p347_fpga_info.dchain[i].dst_start = tpi.ABUF[i].dma_base;
#endif
    }

#ifdef USE_DMA_CHAIN
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	if (i == (p347_ADC_CHANNELS_CNT - 1)) {//Last in chain
	    //p347_fpga_info.dchain[i].sync_mode = OMAP_DMA_SYNC_BLOCK;
	    omap_enable_dma_irq(p347_fpga_info.chain_channel[i],OMAP_DMA_BLOCK_IRQ);
	} else {//no interrupt
	    //reg_val = __raw_readl(dmareg_base+p347_fpga_info.dma_ch*0x60+0x84); //CLINK_CTRLi
	    reg_val = 0x00008000 + p347_fpga_info.chain_channel[i+1]; //ENABLE_LNK + next channel number
	    __raw_writel(reg_val,dmareg_base+p347_fpga_info.chain_channel[i]*0x60+0x84);
	}
    
	omap_set_dma_params(p347_fpga_info.chain_channel[i],&p347_fpga_info.dchain[i]);
	omap_set_dma_src_burst_mode(p347_fpga_info.chain_channel[i],OMAP_DMA_DATA_BURST_8);
	omap_set_dma_dest_burst_mode(p347_fpga_info.chain_channel[i],OMAP_DMA_DATA_BURST_8);
	
	reg_val = __raw_readl(dmareg_base+p347_fpga_info.chain_channel[i]*0x60+0x90); //CSDP2
	reg_val |= 0x40; //bit6 - source provided packed data
	__raw_writel(reg_val,dmareg_base+p347_fpga_info.chain_channel[i]*0x60+0x90);
    
	reg_val = __raw_readl(dmareg_base+p347_fpga_info.chain_channel[i]*0x60+0x80); //CCR2
	//_PDBA("DMA4_CCR v1 = 0x%08x\n",reg_val);
	//reg_val |= 0x20; //bit5 - FS (copy by frame)
	reg_val |= 0x40; //bit6 - read priority to high
	reg_val |= 0x04000000; //bit26 - write priority to high
	reg_val |= 0x01000000; //bit24 - sync by src
	//reg_val |= 0x02000000; //bit25 - buffering disable
	__raw_writel(reg_val,dmareg_base+p347_fpga_info.chain_channel[i]*0x60+0x80);
	reg_val = __raw_readl(dmareg_base+p347_fpga_info.chain_channel[i]*0x60+0x80);
	//_PDBA("DMA4_CCR v2 = 0x%08x\n",reg_val);
	
	tpi.ABUF[i].dma_cur = tpi.ABUF[i].dma_base;
    }
#else
    //!USE_DMA_CHAIN
    //-----------------------------------------------------------dma setup
    p347_fpga_info.dma_par.data_type = OMAP_DMA_DATA_TYPE_S16;
    p347_fpga_info.dma_par.src_start = fpga_mem_read_base;
    p347_fpga_info.dma_par.src_amode = OMAP_DMA_AMODE_POST_INC;
    p347_fpga_info.dma_par.dst_amode = OMAP_DMA_AMODE_POST_INC;
    p347_fpga_info.dma_par.dst_ei = 1;
    p347_fpga_info.dma_par.dst_fi = 1;
    p347_fpga_info.dma_par.elem_count = 8;

    p347_fpga_info.dma_par.dst_start = p347_fpga_info.dma_rd_addr;
    _PDBA("dst addr = 0x%08lx\n",p347_fpga_info.dma_par.dst_start);


#ifdef DMA_DIRECT
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++)
	tpi.ABUF[i].dma_cur = tpi.ABUF[i].dma_base;
    p347_fpga_info.dma_par.frame_count = FRAMES_FOR_SINGLE_CHANNEL;
    p347_fpga_info.dma_par.sync_mode = OMAP_DMA_SYNC_BLOCK;

#else
    //p347_fpga_info.dma_par.src_or_dst_sync = OMAP_DMA_SRC_SYNC;
    if (DATA_FRAME_SIZE > 1) {
	    p347_fpga_info.dma_par.frame_count = DATA_FRAME_SIZE;
	    p347_fpga_info.dma_par.sync_mode = OMAP_DMA_SYNC_BLOCK;
    } else {
	//One 8-words pack
	    p347_fpga_info.dma_par.frame_count = 1;
	    p347_fpga_info.dma_par.sync_mode = OMAP_DMA_SYNC_FRAME;
    }
#endif

    p347_fpga_info.dma_ch = -1;
    ret = omap_request_dma(0,"p347_fpga_read_dma",p347_fpga_dma_callback,&p347_fpga_info,&p347_fpga_info.dma_ch);
    //ret = omap_request_dma(77,"p347_fpga_read_dma",p347_fpga_dma_callback,&p347_fpga_info,&p347_fpga_info.dma_ch);
    _PDBA("omap_request_dma ret %d, channel = %d\n",ret,p347_fpga_info.dma_ch);
    
    omap_set_dma_params(p347_fpga_info.dma_ch,&p347_fpga_info.dma_par);
    omap_set_dma_src_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_8);
    omap_set_dma_dest_burst_mode(p347_fpga_info.dma_ch,OMAP_DMA_DATA_BURST_8);
    
    reg_val = __raw_readl(dmareg_base+p347_fpga_info.dma_ch*0x60+0x90); //CSDP2
    reg_val |= 0x40; //bit6 - source provided packed data
    __raw_writel(reg_val,dmareg_base+p347_fpga_info.dma_ch*0x60+0x90);
    
    reg_val = __raw_readl(dmareg_base+p347_fpga_info.dma_ch*0x60+0x80); //CCR2
    _PDBA("DMA4_CCR v1 = 0x%08x\n",reg_val);
    //reg_val |= 0x20; //bit5 - FS (copy by frame)
    reg_val |= 0x40; //bit6 - read priority to high
    reg_val |= 0x04000000; //bit26 - write priority to high
    reg_val |= 0x01000000; //bit24 - sync by src
    //reg_val |= 0x02000000; //bit25 - buffering disable
    __raw_writel(reg_val,dmareg_base+p347_fpga_info.dma_ch*0x60+0x80);
    reg_val = __raw_readl(dmareg_base+p347_fpga_info.dma_ch*0x60+0x80);
    _PDBA("DMA4_CCR v2 = 0x%08x\n",reg_val);
    
    omap_enable_dma_irq(p347_fpga_info.dma_ch,OMAP_DMA_BLOCK_IRQ);
    
#endif
//end of !USE_DMA_CHAIN

#ifdef USE_HI_TASKLET
    tasklet_init(&irq_delayed_task,do_hi_tasklet,0);
#endif

/*
#ifdef DMA_DIRECT
    //reg_val = __raw_readl(dmareg_base+p347_fpga_info.dma_ch*0x60+0x84); //CLINK_CTRLi
    reg_val = 0x00008000 + p347_fpga_info.dma_ch;
    __raw_writel(reg_val,dmareg_base+p347_fpga_info.dma_ch*0x60+0x84);
                                                                        //CNDPi
    reg_val = &tpi.dma_desc[0];
    __raw_writel(reg_val,dmareg_base+p347_fpga_info.dma_ch*0x60+0xd4);
                                                                        //CCDNi
    reg_val = 0;
    __raw_writel(reg_val,dmareg_base+p347_fpga_info.dma_ch*0x60+0xd8);
                                                                        //CDPi
    reg_val = 0x0135;
    __raw_writel(reg_val,dmareg_base+p347_fpga_info.dma_ch*0x60+0xd0);
#endif
*/

    //tmp_addr = dmareg_base + p347_fpga_info.dma_ch*0x60;
//   _PDBA("GCR = 0x%08x\n",__raw_readl(dmareg_base+0x78));
    //_PDBA("CCR2 = 0x%08x\n",__raw_readl(tmp_addr+0x80));
/*    _PDBA("CLINK2 = 0x%08x\n",__raw_readl(tmp_addr+0x84));
    _PDBA("CICR2 = 0x%08x\n",__raw_readl(tmp_addr+0x88));
    _PDBA("CSR2 = 0x%08x\n",__raw_readl(tmp_addr+0x8C));
    _PDBA("CSDP2 = 0x%08x\n",__raw_readl(tmp_addr+0x90));
    _PDBA("CEN2 = 0x%08x\n",__raw_readl(tmp_addr+0x94));
    _PDBA("CFN2 = 0x%08x\n",__raw_readl(tmp_addr+0x98));
    _PDBA("CSSA2 = 0x%08x\n",__raw_readl(tmp_addr+0x9C));
    _PDBA("CDSA2 = 0x%08x\n",__raw_readl(tmp_addr+0xA0));
    _PDBA("CSEL2 = 0x%08x\n",__raw_readl(tmp_addr+0xA4));
    _PDBA("CSFL2 = 0x%08x\n",__raw_readl(tmp_addr+0xA8));
    _PDBA("CDEL2 = 0x%08x\n",__raw_readl(tmp_addr+0xAC));
    _PDBA("CDFL2 = 0x%08x\n",__raw_readl(tmp_addr+0xB0));
    _PDBA("CSAC2 = 0x%08x\n",__raw_readl(tmp_addr+0xB4));
    _PDBA("CDAC2 = 0x%08x\n",__raw_readl(tmp_addr+0xB8));
    _PDBA("CCEN2 = 0x%08x\n",__raw_readl(tmp_addr+0xBC));
    _PDBA("CCFN2 = 0x%08x\n",__raw_readl(tmp_addr+0xC0));
    _PDBA("COLOR2 = 0x%08x\n",__raw_readl(tmp_addr+0xC4));
    _PDBA("CDP2 = 0x%08x\n",__raw_readl(tmp_addr+0xD0));
    _PDBA("CNDP2 = 0x%08x\n",__raw_readl(tmp_addr+0xD4));
    _PDBA("CCDN2 = 0x%08x\n",__raw_readl(tmp_addr+0xD8));
*/

    //set default delays
    tpi.delays.rot_run          = DEFAULT_DELAY_ROT_RUN;
    tpi.delays.adc_set_params1  = DEFAULT_DELAY_ADC_SET_PARAMS1;
    tpi.delays.adc_set_params2  = DEFAULT_DELAY_ADC_SET_PARAMS2;
    tpi.delays.adc_set_params3  = DEFAULT_DELAY_ADC_SET_PARAMS3;
    tpi.delays.adc_run          = DEFAULT_DELAY_ADC_RUN;
    tpi.delays.adc_run_sync     = DEFAULT_DELAY_ADC_RUN_SYNC;

    //register device for user access
    ret = register_chrdev(p347_FPGA_CHARDEV_NUM,p347_FPGA_CHARDEV_NAME,&p347_fpga_fops);
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

    //timer is for precise time measurement
/*
    tpi.dmt = NULL;
    tpi.dmt = omap_dm_timer_request();
    if (tpi.dmt != NULL) {
	omap_dm_timer_set_prescaler(tpi.dmt,1);
	ret = omap_dm_timer_set_source(tpi.dmt,OMAP_TIMER_SRC_SYS_CLK);
	_PDBA("set timer source ret %d\n",ret);
	omap_dm_timer_start(tpi.dmt);
	_PDB("started timer counter\n");
    } else {
	_PDB("cannot start counter\n");
    }
*/

#ifdef SPI_RECEIVE
    INIT_DELAYED_WORK(&spi_work,delayed_function);
#endif

    gpio_set_value(p347_GPIO_FPGA_RESET,0);
    msleep(1);
    _PDB("FPGA_RESET=>1\n");
    gpio_set_value(p347_GPIO_FPGA_RESET,1);

    p347_fpga_info.user_compatibility = COMPATIBILITY_NOT_CHECKED;
    _PDB("p347_fpga_init end\n");
    return 0;
}
device_initcall(p347_fpga_init);

//==========================================================================

static void __exit p347_fpga_exit(void)
{
    int i;
    t_data_buffer *_abuf_;
    _PDB("p347_fpga_exit start\n");
/*
    if (tpi.dmt != NULL) {
	omap_dm_timer_free(tpi.dmt);
    }
*/
#ifdef SPI_RECEIVE
    cancel_delayed_work_sync(&spi_work);
    //tasklet_kill(&p347_spi_tasklet);
#endif

    spi_unregister_driver(&p347_spi_driver);
    unregister_chrdev(p347_FPGA_CHARDEV_NUM,p347_FPGA_CHARDEV_NAME);

#ifdef USE_DMA_CHAIN
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	//_abuf_ = &(tpi.ABUF[i]);
	//dma_unmap_single(NULL,_abuf_->dma_cur,p347_DMA_SIZE,p347_DMA_DIR);
	//dma_sync_single_for_cpu(NULL,_abuf_->dma_cur,p347_DMA_SIZE,p347_DMA_DIR);
	//printk("unmap at exit _abuf_->dma_cur = %08p\n",_abuf_->dma_cur);
	omap_stop_dma(p347_fpga_info.chain_channel[i]);
	omap_free_dma(p347_fpga_info.chain_channel[i]);
    }
#else
    omap_stop_dma(p347_fpga_info.dma_ch);
    //omap_disable_dma_irq(p347_fpga_info.dma_ch);
    omap_free_dma(p347_fpga_info.dma_ch);
    
    for (i=0; i<p347_ADC_CHANNELS_CNT; i++) {
	_delete_adc_buffer((t_data_buffer*)(&tpi.ABUF[i]));
	//_delete_rotlabels_buffer((t_rotlabels_buffer*)(&tpi.ROTLAB[i]));
    }
    //_delete_timestamps_buffer((t_timestamps_buffer*)(&tpi.TIMES));
    
#ifndef DMA_DIRECT
    dma_free_coherent(NULL,RD_BURST_LEN*2,d_ret,p347_fpga_info.dma_rd_addr);
#endif
#endif
    free_irq(p347_fpga_info.irqnum,&p347_fpga_info);

    //gpio_free(p347_GPIO_FPGA_CS);
    gpio_free(p347_GPIO_FPGA_IRQ);
    gpmc_cs_free(p347_GPMC_FPGA_CS_READ);

    gpio_free(p347_GPIO_FPGA_RESET);
    gpio_free(p347_GPIO_FPGA_DONE);
    gpio_free(p347_GPIO_FPGA_INIT);

#ifdef CS67_MODE
    gpio_free(p347_GPIO_OSCIRQ);
#ifndef USE_CS7_AS_CS
    gpio_free(p347_GPIO_OSCDMA);
#endif
#else
    gpmc_cs_free(p347_GPMC_FPGA_CS_WRITE);
#endif
#ifdef USE_HI_TASKLET
    tasklet_kill(&irq_delayed_task);
#endif
    //gpio_free(p347_GENERATOR_GPIO);
    gpio_free(p347_DISPATCH_GPIO);
    //gpio_free(p347_DATACOPY_GPIO);
    
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
MODULE_AUTHOR(PROGRAM_AUTHOR);
MODULE_LICENSE("GPL");
//==========================================================================
