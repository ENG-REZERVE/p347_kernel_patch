#ifndef _P347_FPGA_H_
#define _P347_FPGA_H_

#include <stdarg.h>

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
#include <linux/pid.h>

#include <plat/gpmc.h>
#include <plat/dma.h>
#include <plat/mcspi.h>
#include <plat/dmtimer.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/p347_fpga_user.h>

#define USE_CS7_AS_CS

//#define YURI_BOARD - deprecated

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
//beeper pin
#define p347_GPIO_BEEP			    13
#define PADCONF_BEEP_ADDR		    0x25d8
#define PADCONF_BEEP_OFFSET		    16
#define PADCONF_BEEP_VALUE		    0x0004
//fpga checking pins
#define p347_GPIO_FPGA_DONE         99
#define p347_FPGA_DONE_BITPOS	    3
#define PADCONF_FPGA_DONE_ADDR      0x2114
#define PADCONF_FPGA_DONE_OFFSET    16
#define PADCONF_FPGA_DONE_VALUE     0x011C

#define p347_GPIO_FPGA_INIT         100
#define p347_FPGA_INIT_BITPOS       4
#define PADCONF_FPGA_INIT_ADDR      0x2118
#define PADCONF_FPGA_INIT_OFFSET    0
#define PADCONF_FPGA_INIT_VALUE     0x011C



#define p347_DISPATCH_GPIO		12
#define PADCONF_DISPATCH_ADDR		0x25d8
#define PADCONF_DISPATCH_OFFSET		0
#define PADCONF_DISPATCH_VALUE		0x0004

#define p347_DATACOPY_GPIO		18
#define PADCONF_DATACOPY_ADDR		0x25e4
#define PADCONF_DATACOPY_OFFSET		0
#define PADCONF_DATACOPY_VALUE		0x0004

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
#ifdef USE_CS7_AS_CS
#define p347_GPMC_FPGA_CS_READ		7
#else
#define p347_GPMC_FPGA_CS_READ		4
#endif

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

#define DMA_DIRECT
//#undef DMA_DIRECT

#define USE_DMA_CHAIN
//#undef USE_DMA_CHAIN

#define COMPATIBILITY_NOT_CHECKED	0
#define COMPATIBILITY_FALSE		1
#define COMPATIBILITY_TRUE		2

//Structure for hardware interfaces
typedef struct
{
    //----------------------------------------------------------GPMC
    unsigned short			DST[RD_BURST_LEN];	//for READ_NODMA mode
    dma_addr_t				dma_rd_addr;
    unsigned short			dma_rd_size;
    atomic_t 				i_cnt; 		//interrupts counter (for debug purposes)
    unsigned long			err_cnt;
    int					opened;
    int					user_compatibility;
    int					irqnum;
    int					dma_ch;
    struct omap_dma_channel_params	dma_par;
#ifdef USE_DMA_CHAIN
    int					chain_channel[p347_ADC_CHANNELS_CNT];
    struct omap_dma_channel_params	dchain[p347_ADC_CHANNELS_CNT];
#endif
    //----------------------------------------------------------SPI
    struct spi_device			*spidev;	//obtain from probe
    struct spi_message 			msg;
    struct spi_transfer			transfer;
    unsigned long			new_hz;
    u32					*spi_tx_buff;
    u32					*spi_rx_buff;
    dma_addr_t				spi_tx_dma;
    dma_addr_t				spi_rx_dma;
}p347_fpga_device_t;

//incoming data buffer for adc channel
typedef struct {
    atomic_t		is_running;
    unsigned short	*ptr;
    atomic_t		write_pos;	//in words
    atomic_t		read_pos;       //in words
    atomic_t		proc_pos;
    atomic_t		cur_len;        //in words, length of uncopied data
    atomic_t		proc_len;	//in words, length of unpocessed data
    int			need_len;	//requested from user, in words
    int			usr_len;	//length of user buffer, for shifting
    spinlock_t		lock;		//for buffer protection
    t_adc_params	par;
#ifdef DMA_DIRECT
    dma_addr_t		dma_base;
    dma_addr_t      dma_cur;
#endif
    unsigned short	test_cnt;
    unsigned long	first_sample_timestamp;
    atomic_t        overflow;
}t_data_buffer;

typedef struct {
    unsigned short	*ptr;
    atomic_t		write_pos;	//in stamps
    atomic_t		read_pos;	//in stamps
    atomic_t		cur_len;	//in stamps
    int			need_len;
#ifdef DMA_DIRECT
    dma_addr_t		dma_base;
    dma_addr_t      dma_cur;
#endif
}t_timestamps_buffer;

/*
//TODO: is should be programmatically setupable
#define MINIMUM_LABELS_DISTANCE			30

typedef struct {
    unsigned short	*ptr;		//offset in adc samples array that corresponds to current label
    atomic_t		write_pos;
    atomic_t		read_pos;
    atomic_t		cur_len;
    unsigned long	prev_sample_index;	//previsious detected rot label index
    unsigned long	label_distance;		//current distances between two rot labels
}t_rotlabels_buffer;
*/
/*
typedef struct {
    unsigned long       dst_addr;
    unsigned long       src_addr;
    unsigned long       elem_num;
    unsigned long       next_ptr;
}t_dma_desc_type3;
*/
//inner handling structure
typedef struct {
    int			        client_pid;
    //struct sock 	*nl_socket;
    //struct nlmsghdr	*nlh;
    //struct sk_buff	*skb;
    //------------------------thread stuff
    //struct task_struct	*datapump_task;
    t_data_buffer	    ABUF[p347_ADC_CHANNELS_CNT];
    //t_rotlabels_buffer	ROTLAB[p347_ADC_CHANNELS_CNT];
    t_fpga_params	    fparam;
    t_timestamps_buffer	TIMES;
    atomic_t		    is_rot_running[p347_ROT_CHANNELS_CNT];
    //t_timestamps_buffer	TIMES[p347_ROT_CHANNELS_CNT];
    spinlock_t		    rot_lock;	//for buffer protection
    unsigned short	    deb_irq_print;
    //------------------------
    //struct omap_dm_timer	*dmt;
    unsigned short      cur_desc;
    t_driver_timings    delays;
    //t_dma_desc_type3    dma_desc[4];
    atomic_t		dma_delayed;
    atomic_t		dma_used;
}t_p347_inner;

#endif
