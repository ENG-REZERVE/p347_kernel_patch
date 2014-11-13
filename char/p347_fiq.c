/*
    
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <asm/fiq.h>
#include <asm/io.h>

#include <plat/gpmc.h>
#include <plat/dma.h>

#define _FIQ_DEBUG_

#ifdef _FIQ_DEBUG_
#define _PDBA(_x_, args...)	printk("_PDB: "_x_, args)
#define _PDB(_x_)		printk("_PDB: "_x_)
#else
#define _PDBA(_x_, args...)	do {}
#define _PDB(_x_)		do {}
#endif

#define p347_FIQ_DRV_VERSION		"2013.12.13#001"

#define p347_GPIO_FPGA_IRQ		    105
#define p347_GPIO_BITPOS		    9
#define PADCONF_FPGA_IRQ_ADDR		0x2120
#define PADCONF_FPGA_IRQ_OFFSET		16
#define PADCONF_FPGA_IRQ_VALUE		0x011C

#define p347_GPIO_OSCIRQ		    57
#define PADCONF_OSCIRQ			    0x20B8
#define PADCONF_OSCIRQ_OFFSET		16
#define PADCONF_OSCIRQ_VALUE		0x0004

//additional FPGA reset pin
#define p347_GPIO_FPGA_RESET		21
#define PADCONF_FPGA_RESET_ADDR		0x25e8
#define PADCONF_FPGA_RESET_OFFSET	16
#define PADCONF_FPGA_RESET_VALUE	0x0004

#define GPIO4_IRQSTATUS             0x49054018

static struct fiq_handler fh = {
    .name		= "p347_fpga_irq",
};

static int fh_id = 0;
static int i_cnt = 0;
static int irq_number = 0;

// r12 - pointers
// r13 - irq status
/*
static void p347_fiq_handler(void) {
    asm volatile(".global p347_fiqs_start\n p347_fiqs_start:");

    // clear gpio irq
    //asm("ldr r12, GPIO4_IRQSTATUS");
    //asm("ldr r13, [#GPIO4_IRQSTATUS]");
    asm("orr r13, #0x200");
    //asm("str r13, [#GPIO4_IRQSTATUS]");

    //asm("ldr r10, GPIO_BASE_ISR");
    //asm("ldr r9, [r10]");
    //asm("orr r9, #0x04");
    //asm("str r9, [r10]");

    //increment int_cnt
    
    //check if dma active or not
    
    //if active, increment delay_counter
    
    //if not active, start dma and set gpio

    //return from fiq
    asm("subs	pc, lr, #4");
    asm(".global p347_fiqs_end\n p347_fiqs_end:");
}
*/

int p347_fiq_enable(void) {
    enable_fiq(irq_number);
    return 0;
}

int p347_fiq_disable(void) {
    disable_fiq(irq_number);
    return 0;
}

extern unsigned char p347_fiq_start;
extern unsigned char p347_fiq_end;

static int __init p347_fiq_init(void)
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

    _PDBA("p347_fiq_init start, version = %s\n",p347_FIQ_DRV_VERSION);
    _PDBA("gpio_base = 0x%08x\n",gpio_base);    
    _PDBA("mpuintc_base = 0x%08x\n",mpuintc_base);   

    i_cnt = 0;
   
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
        msleep(1);
        gpio_set_value(p347_GPIO_FPGA_RESET,0);
	    msleep(1);
	    gpio_set_value(p347_GPIO_FPGA_RESET,1);
        msleep(1);
        //gpio_free(p347_GPIO_FPGA_RESET);

    //-----------------------------------------------------------fpga_irq pad configuration
    padconf_addr = scm_base + PADCONF_FPGA_IRQ_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    if (PADCONF_FPGA_IRQ_OFFSET) reg_val &= 0x0000FFFF; else reg_val &= 0xFFFF0000;
    reg_val |= (PADCONF_FPGA_IRQ_VALUE << PADCONF_FPGA_IRQ_OFFSET);
    __raw_writel(reg_val,padconf_addr);
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);

    irq_number = gpio_to_irq(p347_GPIO_FPGA_IRQ);
    _PDBA("gpio_to_irq ret irqnum=%d\n",irq_number);

    ret = gpio_request(p347_GPIO_FPGA_IRQ,"gpio_fpga_irq");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_fpga_irq, ret=%d\n",ret);
	return ret;
    }
    _PDB("gpio_fpga_irq registered\n");
    


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


    ret = claim_fiq(&fh);
    if (ret) {
	_PDBA("Failed to claim FIQ, err = %d\n",ret);
    } else {
	    set_fiq_handler(&p347_fiq_start,&p347_fiq_start - &p347_fiq_end);
	    //set_fiq_regs(&regs);

        //redirect to fiq
        tmp_addr = mpuintc_base + 0x100 + 4*INT_34XX_GPIO_BANK4;
        reg_val = __raw_readl(tmp_addr) | 0x1;
        __raw_writel(reg_val,tmp_addr);
        reg_val = __raw_readl(tmp_addr);
        _PDBA("IRQ %d redirected to FIQ, ILR reg=%08x\n",irq_number,reg_val);

        enable_fiq(irq_number);
    }

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
/*
    //register device for user access
    ret = register_chrdev(p347_FIQ_CHARDEV_NUM,p347_FIQ_CHARDEV_NAME,&p347_fpga_fops);
    if (ret < 0) {
	    gpio_free(p347_GPIO_FPGA_IRQ);
	    _PDB("ERROR: cannot register char device for p347_fpga\n");
	    return ret;
    }
*/
    padconf_addr = scm_base + PADCONF_FPGA_IRQ_ADDR;
    reg_val = __raw_readl(padconf_addr);
    _PDBA("read irq padconf 0x%p ret 0x%x\n",(void*)padconf_addr,reg_val);
    //-----------------------------------------------------------

    _PDB("p347_fiq_init end\n");
    return 0;
}
arch_initcall(p347_fiq_init);


static void __exit p347_fiq_exit(void)
{
    gpio_free(p347_GPIO_FPGA_IRQ);
}

module_exit(p347_fiq_exit);
//==========================================================================
MODULE_AUTHOR("Konstantin Utkin");
MODULE_LICENSE("GPL");
//==========================================================================
