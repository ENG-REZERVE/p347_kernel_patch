#ifndef _P347_FPGA_USER_H_
#define _P347_FPGA_USER_H_

#include "p347_err.h"

/*
27.06.2014: removed timestamps reading from adc_run,
    added fpga and adc registers verifying,
    added function of fpga status checking
05.06.2014: added channel power-off after invalid setup
*/

#define VERSION_MAJOR			3
//#define VERSION_MINOR			140428
#define VERSION_MINOR			140627
#define VERSION_TAIL			2

#define p347_FPGA_CHARDEV_NAME		"p347_fpga_device"
#define p347_FPGA_CHARDEV_NUM		241

#define ADC_DATA_SIZE_WORDS		2
#define ADC_DATA_SIZE_BYTES		ADC_DATA_SIZE_WORDS*2
#define ROT_DATA_SIZE_WORDS		2
#define ROT_DATA_SIZE_BYTES		ROT_DATA_SIZE_WORDS*2

#define DATA_FRAME_SIZE			512
#define FRAMES_FOR_SINGLE_CHANNEL	128

#define CS67_MODE
//mode when gpmc CS6 and CS7 are outputs to oscilloscope,
//irq_handler uses CS6 and dma_handler uses CS7
//#define MINIRQ_MODE
//minimize IRQ quantity: use it only for start

#define p347_ADC_CHANNELS_CNT		4
#define p347_ROT_CHANNELS_CNT		4

#define OFFSET_OF(field_name, struct_name)	(&(((struct_name *)NULL)->field_name))

#define ROT_BIT_MASK			0x2000

//---------------------------------------------------------------------------------

typedef struct {
    unsigned long	major;
    unsigned long	minor;
    unsigned long	tail;
    unsigned long	reserved;
}t_version;

// t_gpmc_data structure used only for raw r-w access to gpmc,
// called from ioctl, not used in actual protocol.
typedef struct {
    unsigned int	is_dma;
    unsigned int	word_cnt;
    unsigned short	*data;
}t_gpmc_data;

//t_spi_data used in all ioctl functions that calls spi transfers
typedef struct {
    unsigned int	is_dma;
    unsigned int	word_cnt;
    unsigned long	*data_tx;
    unsigned long	*data_rx;
}t_spi_data;

//loading signal from user to fpga
typedef struct {
    unsigned long	*u_ptr;
    unsigned long	len;
}t_signal_load;

//user must fill this parameters
typedef struct {
    unsigned short	control1;
    unsigned short	control2; //proper power-on command
    unsigned short	offset;
    unsigned short	gain;
    unsigned short	overrange;
    //unsigned short	filt_coeff;
    unsigned short	ch_settings;
}t_adc_params;

typedef struct {
    unsigned char   rot_idx;
    //1bit = 6.25ns 
    unsigned long   pulse_width_min;
    unsigned long   period_min;
    unsigned long   period_max;
}t_rot_params;

typedef struct {
    unsigned char 	idx;
    unsigned long	time_offset;
}t_adc_start_single;

//structure for setup_channel ioctl call
typedef struct {
    unsigned char	idx;
    unsigned char	rot_idx;
    int			len; //measured in data packs, not bytes
    int			usr_len;
    t_adc_params	apar;
}t_ch_params;

//structure for copy_adc ioctl call
typedef struct {
    unsigned long*	u_ptr;
    unsigned char	ch_idx;
    //unsigned long	fstamp;
}t_transfer_params;

typedef struct {
    unsigned char	adc_ch_cnt;
    unsigned char	adc_ch_idx[p347_ADC_CHANNELS_CNT];
    unsigned long	adc_ch_offset[p347_ADC_CHANNELS_CNT];
    unsigned long   sync_reg;
}t_synctask_channels;

typedef struct {
    unsigned short	settings;
    unsigned short  rot_ch_mapping;
}t_fpga_params;

typedef struct {
    unsigned short	us_width;
    unsigned short	us_idle;
    unsigned long	count;
}t_selfgeneration;

typedef struct {
    unsigned short  rot_run;            //default=1000
    unsigned short  adc_set_params1;    //default=1000
    unsigned short  adc_set_params2;    //default=100
    unsigned short  adc_set_params3;    //default=1
    unsigned short  adc_run;            //default=1
    unsigned short  adc_run_sync;       //default=2500
}t_driver_timings;

#define DEFAULT_DELAY_ROT_RUN           1000
#define DEFAULT_DELAY_ADC_SET_PARAMS1   1000
#define DEFAULT_DELAY_ADC_SET_PARAMS2   100
#define DEFAULT_DELAY_ADC_SET_PARAMS3   1
#define DEFAULT_DELAY_ADC_RUN           1
#define DEFAULT_DELAY_ADC_RUN_SYNC      2500

#define ADC_COEFS_VARIATIONS_NUMBER	8
#define MAX_ADC_COEFS_NUMBER		49

//static unsigned char ac_variations[ADC_COEFS_VARIATIONS_NUMBER] = {6,12,18,24,30,36,42,48};
static unsigned char ac_variations[ADC_COEFS_VARIATIONS_NUMBER] = {7,13,19,25,31,37,43,49};
static unsigned char flen_variations[ADC_COEFS_VARIATIONS_NUMBER] = {0x1,0x3,0x5,0x7,0x9,0xA,0xC,0xF};

typedef struct {
    unsigned char	ch_idx;
    unsigned char	flen;
    int			coefs_number;
    unsigned long	coefs[MAX_ADC_COEFS_NUMBER];
}t_adc_coefs;

//!!! MAXRECORDS*4 should be = N*page_size (N*4096 bytes)
//TODO: add special runtime check for page size and maxrecords

//Pump Buffer is for adc data for each channel
//24Mb
//#define p347_PUMPBUF_MAXRECORDS		6291456
//0x1800000 bytes minus 1 page
#define p347_PUMPBUF_MAXRECORDS		6290432
#define ADC_SAMPLES_IN_BUFFER		p347_PUMPBUF_MAXRECORDS

//memory in words
#define p347_PUMPBUF_FULLSIZE		2*p347_PUMPBUF_MAXRECORDS
//memory in bytes
#define p347_PUMPBUF_BYTESIZE		4*p347_PUMPBUF_MAXRECORDS

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

#define p347_SPI_MIN_SPEED			p347_SPI_SPEED_1_5kHZ

//TESTING IOCTL
#define p347_IOCTL_GET_INT_CNT			03
#define p347_IOCTL_GENERATE_PULSE		04
#define p347_IOCTL_SET_DEBUG_PRINTS		05
#define p347_IOCTL_FPGA_RESET			06

//GPMC IOCTL (raw)
#define p347_IOCTL_WRITE			10
#define p347_IOCTL_READ				11

//SPI IOCTL
#define p347_IOCTL_SPI_WRITE			20
#define p347_IOCTL_SPI_SENDCMD			21
#define p347_IOCTL_SPI_SET_HZ			22
#define p347_IOCTL_SPI_READ_REGISTER		23
#define p347_IOCTL_SPI_WRITE_REGISTER		24

//Client Interface IOCTL
#define p347_IOCTL_CLIENT_CHECK_FPGA_STATUS	30
#define p347_IOCTL_CLIENT_SET_DRIVER_DELAYS 	31
#define p347_IOCTL_CLIENT_SETUP_CHANNEL		32
#define p347_IOCTL_CLIENT_COPY_ADC		33
//#define p347_IOCTL_CLIENT_COPY_TIMESTAMPS	34
#define p347_IOCTL_CLIENT_CHECK_DATA		35
#define p347_IOCTL_CLIENT_START_CHANNEL		36
#define p347_IOCTL_CLIENT_STOP_CHANNEL		37
#define p347_IOCTL_CLIENT_START_SYNC		38
#define p347_IOCTL_CLIENT_STOP_SYNC		39
#define p347_IOCTL_CLIENT_SET_CC		40
//#define p347_IOCTL_CLIENT_COPY_ROTLABELS	41
#define p347_IOCTL_CLIENT_LOAD_SIGNAL		42
#define p347_IOCTL_CLIENT_START_ROT		43
#define p347_IOCTL_CLIENT_STOP_ROT		44
#define p347_IOCTL_CLIENT_READ_ROT 		45
#define p347_IOCTL_CLIENT_GET_ADDRESSES		46
#define p347_IOCTL_CLIENT_GET_BASE_ADDRESSES	47
#define p347_IOCTL_CLIENT_SHIFT_RPOS		48
#define p347_IOCTL_CLIENT_LOAD_ADC_COEFS	49
#define p347_IOCTL_CLIENT_SHIFT_PROCPOS		51
#define p347_IOCTL_CLIENT_READ_OFFSETS		52
#define p347_IOCTL_CLIENT_ATTACH_BUFFERS	53
#define p347_IOCTL_CLIENT_DETACH_BUFFERS	54
#define p347_IOCTL_CLIENT_WARM_UP		55
#define p347_IOCTL_CLIENT_WARM_OFF		56
#define p347_IOCTL_CLIENT_CHECK_VERSION		57

//Testing IOCTL
#define p347_IOCTL_TEST_ASM			60

//ADC Registers
#define p347_ADC_REG_CONTROL_1			0x1
#define p347_ADC_REG_CONTROL_2			0x2
#define ADC_POWER_ON                    	0x0002
#define ADC_POWER_ON_ESAVE                    	0x0006
#define ADC_POWER_OFF                   	0x000A
#define p347_ADC_REG_OFFSET			0x3
#define p347_ADC_REG_GAIN			0x4
#define p347_ADC_REG_OVERRANGE			0x5
#define p347_ADC_REG_COEFF_LOW			0x6
#define p347_ADC_REG_COEFF_HIGH			0x7
#define p347_ADC_REG_CH_SETTINGS		0x8
//FPGA Registers
#define p347_FPGA_REG_SETTINGS			0x9
#define p347_FPGA_REG_STATUS			0xA
#define p347_FPGA_REG_SYNC			0xB
#define p347_FPGA_REG_TIMESTAMP1		0xC
#define p347_FPGA_REG_TIMESTAMP2		0xD
#define p347_FPGA_REG_TIMESTAMP3		0xE
#define p347_FPGA_REG_TIMESTAMP4		0xF
#define p347_FPGA_REG_DO_WIDTH_MIN_LOW	    0x10
#define p347_FPGA_REG_DO_WIDTH_MIN_HIGH	    0x11
#define p347_FPGA_REG_DO_PERIOD_MIN_LOW     0x12
#define p347_FPGA_REG_DO_PERIOD_MIN_HIGH    0x13
#define p347_FPGA_REG_DO_PERIOD_MAX_LOW     0x14
#define p347_FPGA_REG_DO_PERIOD_MAX_HIGH    0x15
#define p347_FPGA_REG_DO1               0x16
#define p347_FPGA_REG_DO2               0x17
#define p347_FPGA_REG_DO3               0x18
#define p347_FPGA_REG_DO4               0x19
#define p347_FPGA_REG_ROT_CH_MAPPING    0x20

#define p347_MAX_REG_NUM			    0x20

//SPI Magic Long
#define p347_SPI_MAGIC_LONG			0xC0DEBEEF

#endif
