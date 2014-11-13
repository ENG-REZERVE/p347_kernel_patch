#ifndef _P347_FPGA_H_
#define _P347_FPFA_H_

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/signal.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>

#include <linux/security.h>

#include <plat/mcspi.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/p347_fpga_user.h>

#define YURI_BOARD

#ifdef YURI_BOARD
/*
//gpio pin number for IRQ
#define p347_GPIO_FPGA_IRQ		20
//bit position in GPIO block
#define p347_GPIO_BITPOS		19
//mux mode and input enable register for pin requested for IRQ
#define PADCONF_FPGA_IRQ_ADDR		0x25E8
//offset in PADCONF register (high word or low word)
#define PADCONF_FPGA_IRQ_OFFSET		0
//value for irq conf
//#define PADCONF_FPGA_IRQ_VALUE		0x0104
#define PADCONF_FPGA_IRQ_VALUE		0x011C
*/
#define p347_GPIO_FPGA_IRQ		105
#define p347_GPIO_BITPOS		9
#define PADCONF_FPGA_IRQ_ADDR		0x2120
#define PADCONF_FPGA_IRQ_OFFSET		16
#define PADCONF_FPGA_IRQ_VALUE		0x011C
//additional FPGA reset pin
#define p347_GPIO_FPGA_RESET		21
#define PADCONF_FPGA_RESET_ADDR		0x25e8
#define PADCONF_FPGA_RESET_OFFSET	16
#define PADCONF_FPGA_RESET_VALUE	0x0004
#else
#define p347_GPIO_FPGA_IRQ		111
#define p347_GPIO_BITPOS		15
#define PADCONF_FPGA_IRQ_ADDR		0x212C
#define PADCONF_FPGA_IRQ_OFFSET		16
#define PADCONF_FPGA_IRQ_VALUE		0x011C
#endif

//gpio pin for oscilloscope irq toggling (gpmc cs6)
#define p347_GPIO_OSCIRQ		57
#define PADCONF_OSCIRQ			0x20B8
#define PADCONF_OSCIRQ_OFFSET		16
#define PADCONF_OSCIRQ_VALUE		0x0004
//gpio pin for oscilloscope dma toggling (gpmc cs7)
#define p347_GPIO_OSCDMA		58
#define PADCONF_OSCDMA			0x20BC
#define PADCONF_OSCDMA_OFFSET		0
#define PADCONF_OSCDMA_VALUE		0x0004

//padconf addr for gmpc_clk pin
#define PADCONF_GPMC_CLK_ADDR		0x20BC
//mcspi3_padconf
#define PADCONF_SPI3_SIMO_ADDR		0x25DC
#define PADCONF_SPI3_SIMO_OFFSET	0
#define PADCONF_SPI3_SIMO_VALUE		0x0019
#define PADCONF_SPI3_SOMI_ADDR		0x25DC
#define PADCONF_SPI3_SOMI_OFFSET	16
#define PADCONF_SPI3_SOMI_VALUE		0x0119
#define PADCONF_SPI3_CS1_ADDR		0x25E8
#define PADCONF_SPI3_CS1_OFFSET		16
#define PADCONF_SPI3_CS1_VALUE		0x0019
#define PADCONF_SPI3_CLK_ADDR		0x25E0
#define PADCONF_SPI3_CLK_OFFSET		16
#define PADCONF_SPI3_CLK_VALUE		0x0119
//#define PADCONF_SPI3_CLK_VALUE		0x0019
#define PADCONF_SPI3_CS0_ADDR		0x25E0
#define PADCONF_SPI3_CS0_OFFSET		0
#define PADCONF_SPI3_CS0_VALUE		0x0019

#define p347_GPMC_FPGA_CS_WRITE		6
#define p347_GPMC_FPGA_CS_READ		4

#define _P347_DEBUG_

#ifdef _P347_DEBUG_
#define _PDBA(_x_, args...)	printk("_PDB: "_x_, args)
#define _PDB(_x_)		printk("_PDB: "_x_)
#else
#define _PDBA(_x_, args...)	do {}
#define _PDB(_x_)		do {}
#endif

#define RD_BURST_LEN			8

//spi buffers length in words
#define SPI_RXBUFF_LEN			16
#define SPI_TXBUFF_LEN			16

//Structure for hardware interfaces
typedef struct
{
    //----------------------------------------------------------GPMC
    //unsigned short			DST[RD_BURST_LEN];	//for READ_NODMA mode
    //dma_addr_t				dma_rd_addr;
    //unsigned short			dma_rd_size;
    unsigned long 			i_cnt; 		//interrupts counter (for debug purposes)
    unsigned long			err_cnt;
    int					opened;
    int					irqnum;
    //int					dma_ch;
    //struct omap_dma_channel_params	dma_par;
    //----------------------------------------------------------SPI
    struct spi_device			*spidev;	//obtain from probe
    struct spi_message 			msg;
    struct spi_transfer			transfer;
    unsigned long			new_hz;
    u32					*spi_tx_buff;
    u32					*spi_rx_buff;
}p347_fpga_device_t;

//inner handling structure
typedef struct {
    //int			client_pid;
    //struct sock 	*nl_socket;
    //struct nlmsghdr	*nlh;
    //struct sk_buff	*skb;
    //------------------------thread stuff
    //struct task_struct	*datapump_task;
    //t_data_buffer	ABUF[p347_ADC_CHANNELS_CNT];
    t_fpga_params	fparam;
    //t_rot_buffer	ROT;
    //------------------------
    //struct omap_dm_timer	*dmt;
}t_p347_inner;

#endif
