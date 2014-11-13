#ifndef _P347_FPGA_USER_H_
#define _P347_FPFA_USER_H_

#define p347_CHARDEV_NAME		"p347_fpga_device"
#define p347_CHARDEV_NUM		241

typedef struct {
    unsigned int	is_dma;
    unsigned int	word_cnt;
    unsigned short	*data;
}t_gpmc_data;

typedef struct {
    unsigned int	is_dma;
    unsigned int	word_cnt;
    unsigned long	*data_tx;
    unsigned long	*data_rx;
}t_spi_data;

//default spi speeds, should be used in IOCTL_SET_HZ calls
#define p347_SPI_SPEED_48MHZ		48000000
#define p347_SPI_SPEED_24MHZ		24000000
#define p347_SPI_SPEED_12MHZ		12000000
#define p347_SPI_SPEED_6MHZ		6000000
#define p347_SPI_SPEED_3MHZ		3000000
#define p347_SPI_SPEED_1_5MHZ		1500000
#define p347_SPI_SPEED_750kHZ		750000
#define p347_SPI_SPEED_375kHZ		375000
#define p347_SPI_SPEED_187kHZ		187500
#define p347_SPI_SPEED_94kHZ		93750
#define p347_SPI_SPEED_47kHZ		46875
#define p347_SPI_SPEED_23kHZ		23438
#define p347_SPI_SPEED_12kHZ		11719
#define p347_SPI_SPEED_6kHZ		5859
#define p347_SPI_SPEED_3kHZ		2930
#define p347_SPI_SPEED_1_5kHZ		1465

#define p347_SPI_MIN_SPEED		p347_SPI_SPEED_1_5kHZ

//GPMC IOCTL
#define p347_IOCTL_WRITE		10
#define p347_IOCTL_READ			11

//SPI IOCTL
#define p347_IOCTL_SPI_WRITE		20
#define p347_IOCTL_SPI_SENDCMD		21
#define p347_IOCTL_SPI_SET_HZ		22

//Client Interface IOCTL
#define p347_IOCTL_CLIENT_REGISTER	30
#define p347_IOCTL_CLIENT_UNREGISTER	31

#endif
