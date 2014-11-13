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

#include <plat/gpmc.h>
#include <plat/dma.h>
#include <plat/mcspi.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/p347_fpga_user.h>

//gpio pin number for IRQ
#define p347_GPIO_FPGA_IRQ		111
#define p347_GPIO_BITPOS		15
//mux mode and input enable register for pin requested for IRQ
#define PADCONF_FPGA_IRQ_ADDR		0x212C
//offset in PADCONF register (high word or low word)
#define PADCONF_FPGA_IRQ_OFFSET		16
//value for irq conf
//#define PADCONF_FPGA_IRQ_VALUE		0x0104
#define PADCONF_FPGA_IRQ_VALUE		0x011C
//padconf addr for gmpc_clk pin
#define PADCONF_GPMC_CLK_ADDR		0x20BC

#define p347_GPMC_FPGA_CS		4

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

//TODO: proper structure
typedef struct
{
    //----------------------------------------------------------GPMC
    unsigned short			DST[RD_BURST_LEN];
    dma_addr_t				dma_rd_addr;
    unsigned short			dma_rd_size;
    unsigned long 			i_cnt;
    unsigned long			err_cnt;
    int					opened;
    int					irqnum;
    int					dma_ch;
    struct omap_dma_channel_params	dma_par;
    //----------------------------------------------------------SPI
    struct spi_device			*spidev; //obtain from probe
    struct spi_message 			msg;
    struct spi_transfer			transfer;
    u32					*spi_tx_buff;
    u32					*spi_rx_buff;
}p347_fpga_device_t;

#endif
