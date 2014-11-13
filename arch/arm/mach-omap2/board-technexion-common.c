/*
 * TechNexion Common:
 *
 * This file contains initialization of common peripherals / drivers found
 * on TechNexion modules and carrierboards. 
 *
 * Copyright (C) 2012 TechNexion Inc
 *
 * Maintainer: TechNexion <linuxfae@technexion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <plat/omap-pm.h>

#include <plat/display.h>
#include <plat/usb.h>

#include <mach/hardware.h>
#include <plat/gpmc.h>
#include "mux.h"

#include <linux/interrupt.h>
#include "board-technexion-common.h"

/****************************************************************************
 *
 * ANDROID GADGET
 *
 ****************************************************************************/

#ifdef CONFIG_USB_ANDROID

#include <linux/usb/android_composite.h>

#define GOOGLE_VENDOR_ID		0x18d1
#define GOOGLE_PRODUCT_ID		0x9018
#define GOOGLE_ADB_PRODUCT_ID		0x9015

static char *tncommon_usb_functions_adb[] = {
	"adb",
};

static char *tncommon_usb_functions_mass_storage[] = {
	"usb_mass_storage",
};
static char *tncommon_usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *tncommon_usb_functions_all[] = {
	"adb", "usb_mass_storage",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(tncommon_usb_functions_adb),
		.functions	= tncommon_usb_functions_adb,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(tncommon_usb_functions_mass_storage),
		.functions	= tncommon_usb_functions_mass_storage,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(tncommon_usb_functions_ums_adb),
		.functions	= tncommon_usb_functions_ums_adb,
	},
};

static struct usb_mass_storage_platform_data tncommon_gadget_ums_pdata = {
	.nluns		= 1,
	.vendor		= "TN",
	.product	= "UMS Gadget",
	.release	= 0x100,
};

static struct platform_device tncommon_gadget_ums_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &tncommon_gadget_ums_pdata,
	},
};

static struct android_usb_platform_data tncommon_android_usb_pdata = {
	.vendor_id	= GOOGLE_VENDOR_ID,
	.product_id	= GOOGLE_PRODUCT_ID,
	.functions	= tncommon_usb_functions_all,
	.num_products	= ARRAY_SIZE(usb_products),
	.products	= usb_products,
	.version	= 0x0100,
	.product_name	= "android gadget",
	.manufacturer_name	= "TechNexion",
	.serial_number	= "20100720",
	.num_functions	= ARRAY_SIZE(tncommon_usb_functions_all),
};

static struct platform_device tncommon_androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &tncommon_android_usb_pdata,
	},
};

void tncommon_android_gadget_init(void)
{
	platform_device_register(&tncommon_androidusb_device);
	platform_device_register(&tncommon_gadget_ums_device);
}

#endif

/****************************************************************************
 *
 * OMAP2+ Nand devices
 *
 ****************************************************************************/

#if defined(CONFIG_MTD_NAND_OMAP2) || \
                defined(CONFIG_MTD_NAND_OMAP2_MODULE)

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/physmap.h>
#include <plat/nand.h>
#include <asm/mach/flash.h>

/* Note that all values in this struct are in nanoseconds */
static struct gpmc_timings tncommon_nand_timings = {

        .sync_clk = 0,

        .cs_on = 0,
        .cs_rd_off = 36,
        .cs_wr_off = 36,

        .adv_on = 6,
        .adv_rd_off = 24,
        .adv_wr_off = 36,

        .we_off = 30,
        .oe_off = 48,

        .access = 54,
        .rd_cycle = 72,
        .wr_cycle = 72,

        .wr_access = 30,
        .wr_data_mux_bus = 0,
};

static struct omap_nand_platform_data tncommon_nand_data = {
        .nand_setup     = NULL,
        .gpmc_t         = &tncommon_nand_timings,
        .dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
        .dev_ready      = NULL,
        .devsize        = 0,    /* '0' for 8-bit, '1' for 16-bit device */
};

void __init tncommon_nand_init(struct mtd_partition *nand_parts,
                        u8 nr_parts, u8 cs, int nand_type)
{
	tncommon_nand_data.cs              = cs;
	tncommon_nand_data.parts           = nand_parts;
	tncommon_nand_data.nr_parts        = nr_parts;
	tncommon_nand_data.devsize         = nand_type;

	tncommon_nand_data.gpmc_irq = OMAP_GPMC_IRQ_BASE + cs;
	tncommon_nand_data.ecc_opt = OMAP_ECC_HAMMING_CODE_DEFAULT;
	tncommon_nand_data.xfer_type = NAND_OMAP_POLLED;

	if (cpu_is_omap3517() || cpu_is_omap3505())
		tncommon_nand_data.gpmc_t = NULL;

	gpmc_nand_init(&tncommon_nand_data);
}
#else
void
__init tncommon_nand_init(struct mtd_partition *nand_parts, u8 nr_parts,
                u8 cs, int nand_type)
{
}
#endif /* CONFIG_MTD_NAND_OMAP2 || CONFIG_MTD_NAND_OMAP2_MODULE */


#if defined(CONFIG_MTD_NAND)
#define NAND_BLOCK_SIZE	SZ_128K

/*OAMP3 Flahs Init*/
static struct mtd_partition tncommon_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel image",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 80 * NAND_BLOCK_SIZE,/*32->80*/
	},

	{
		.name		= "Root File System",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},

/*
	{
		.name		= "Root File System",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2000 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "User File System",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
*/
};

/*Function init all nand/nor devices*/
void __init tncommon_flash_devices_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		tncommon_nand_init(tncommon_nand_partitions,
			ARRAY_SIZE(tncommon_nand_partitions),
			nandcs, NAND_BUSWIDTH_16);
	}
}
#endif

/****************************************************************************
 *
 * ADS7846 SPI 4-wire Resistance Touch
 *
 ****************************************************************************/

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
#define A78
#else
#undef A78
#endif

#ifdef A78
/*Finetune Electronic Characters for each TouchScreen*/

struct tncommon_tsc_sensitivity_param {
	char *name;
	u16	x_plate_ohms;
	u16	debounce_max;		/* max number of additional readings per sample */
	u16	debounce_tol;		/* tolerance used for filtering */
	u16	debounce_rep;		/* additional consecutive good readings required after the first two */	
	u16	settle_delay_usecs;
}resist_touch_param;

static struct tncommon_tsc_sensitivity_param tncommon_ads7846_sense_param[] = {
	/*name,		x_plate_ohms	,debounce_max,	debounce_tol,	debounce_rep,	settle_delay_usecs,*/
	{"lb043wq2"		,50		,70			,30		,1		,75	},
	{"at070tn93"		,150		,70			,30		,1		,150	},
	{"default"		,80		,10			,3		,1		,100	}
};

static int tncommon_ads7846_sense_param_index = 0;

#define TNCOMMON_TSC_IRQ_GPIO	136

#endif

#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#ifdef A78
static struct omap2_mcspi_device_config tncommon_ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,/* 0: slave, 1: master */
};

static int tncommon_ads7846_get_pendown_state(void)
{
	return !gpio_get_value(TNCOMMON_TSC_IRQ_GPIO);
}

static struct ads7846_platform_data tncommon_ads7846_config = {
        .x_max              = 0x0fff,
        .y_max                  = 0x0fff,
        .pressure_max           = 255,
        .get_pendown_state      = tncommon_ads7846_get_pendown_state,
        .keep_vref_on           = 1,
        .wakeup                         = true,
};

#endif

//+uki
static struct omap2_mcspi_device_config p347_mcspi_config = {
    .turbo_mode		= 0,
    .single_channel	= 1,/* 0: slave, 1: master */
};

#include <linux/can/platform/mcp251x.h>

static struct mcp251x_platform_data mcp251x_info = {
    .oscillator_frequency = 20000000,
    //.board_specific_setup = &mcp251x_setup,
    .board_specific_setup = NULL,//???
    .transceiver_enable = NULL,
    .power_enable = NULL,
};

#define GPIO_CAN_INTERRUPT		107
#define GPIO_CAN_CHIPSELECT		126

//-uki

static struct spi_board_info tncommon_board_spi_tsc_info[] __initdata = {
#ifdef A78
	 {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &tncommon_ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(TNCOMMON_TSC_IRQ_GPIO),
		.platform_data		= &tncommon_ads7846_config,
	},
#endif
//+uki
	{//for fpga driver
		.modalias		= "p347_spi",
		.bus_num		= 3,
		.chip_select		= 0,
		.max_speed_hz		= 48000000,
		.controller_data	= &p347_mcspi_config,
		.platform_data		= NULL,
	},
	{//for can
		.modalias		= "mcp2515",
		.bus_num		= 3,
		.platform_data		= &mcp251x_info,
		.irq			= OMAP_GPIO_IRQ(GPIO_CAN_INTERRUPT),
		.chip_select		= 1,//TODO: what number actially?
		.max_speed_hz		= 10000000,
	},
//-uki
};

static void __init tncommon_spi_tsc_init(void)
{
#ifdef A78
	tncommon_ads7846_config.x_plate_ohms		= tncommon_ads7846_sense_param[tncommon_ads7846_sense_param_index].x_plate_ohms;
	tncommon_ads7846_config.debounce_max 		  = tncommon_ads7846_sense_param[tncommon_ads7846_sense_param_index].debounce_max;
	tncommon_ads7846_config.debounce_tol 		  = tncommon_ads7846_sense_param[tncommon_ads7846_sense_param_index].debounce_tol;
	tncommon_ads7846_config.debounce_rep 		  = tncommon_ads7846_sense_param[tncommon_ads7846_sense_param_index].debounce_rep;
	tncommon_ads7846_config.settle_delay_usecs	  = tncommon_ads7846_sense_param[tncommon_ads7846_sense_param_index].settle_delay_usecs;

	if ((gpio_request(TNCOMMON_TSC_IRQ_GPIO, "ADS7846 pendown") == 0) &&
	    (gpio_direction_input(TNCOMMON_TSC_IRQ_GPIO) == 0)) {
		gpio_export(TNCOMMON_TSC_IRQ_GPIO, 0);
	} else {
		printk(KERN_ERR "could not obtain gpio for ADS7846_PENDOWN\n");
		return;
	}

	gpio_direction_input(TNCOMMON_TSC_IRQ_GPIO);
	gpio_set_debounce(TNCOMMON_TSC_IRQ_GPIO, 0xa);
#endif
	spi_register_board_info(tncommon_board_spi_tsc_info,
			ARRAY_SIZE(tncommon_board_spi_tsc_info));
}

int tncommon_start_tsc_devices(void)
{
	tncommon_spi_tsc_init();
	return 0;
}

/****************************************************************************
 *
 * SMSC9220 Localbus LAN
 *
 ****************************************************************************/

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)

#include <linux/smsc911x.h>

static __initdata int tncommon_smsc911x_gpio_irq = 1;
static __initdata int tncommon_smsc911x_gpio_reset = 9;
static __initdata int tncommon_smsc911x_cs = 5;

static struct resource tncommon_smsc911x_resources[] = {
	{
		.name	= "smsc911x-memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.flags	=  (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config tncommon_smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags		= (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS ),
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct platform_device tncommon_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tncommon_smsc911x_resources),
	.resource	= &tncommon_smsc911x_resources,
	.dev		= {
		.platform_data = &tncommon_smsc911x_config,
	},
};

static inline void __init tncommon_smsc911x_init(void)
{
	unsigned long cs_mem_base;

	if (gpmc_cs_request(tncommon_smsc911x_cs, SZ_16M, &cs_mem_base) < 0)
	{
		printk(KERN_ERR "Failed request for GPMC mem for smsc911x\n");
		return;
	}
	printk("===========SMSC GPMC CS REQUEST addr = 0x%08x\n",cs_mem_base);

	tncommon_smsc911x_resources[0].start = cs_mem_base + 0x0;
	tncommon_smsc911x_resources[0].end   = cs_mem_base + SZ_16K;

	omap_mux_init_gpio(tncommon_smsc911x_gpio_irq, OMAP_PIN_INPUT_PULLUP|OMAP_MUX_MODE4);

	if ((gpio_request(tncommon_smsc911x_gpio_irq, "SMSC911X IRQ") == 0) &&
			(gpio_direction_input(tncommon_smsc911x_gpio_irq) == 0))
	{
		gpio_export(tncommon_smsc911x_gpio_irq, 0);
	}
	else
	{
		printk(KERN_ERR "could not obtain gpio for SMSC911X IRQ\n");
		return;
	}

	tncommon_smsc911x_resources[1].start = OMAP_GPIO_IRQ(tncommon_smsc911x_gpio_irq);
	tncommon_smsc911x_resources[1].end  = OMAP_GPIO_IRQ(tncommon_smsc911x_gpio_irq);

	omap_mux_init_gpio(tncommon_smsc911x_gpio_reset, OMAP_PIN_INPUT_PULLUP|OMAP_MUX_MODE4);

	if (gpio_request(tncommon_smsc911x_gpio_reset, "smsc911x reset") < 0)
	{
		printk(KERN_ERR "can't get smsc911x reset GPIO\n");
		return;
	}

	printk("resetting lan9220 chip...\n");
	gpio_direction_output(tncommon_smsc911x_gpio_reset,0);
	//mdelay(1);
	mdelay(50);
	gpio_direction_output(tncommon_smsc911x_gpio_reset,1);

	platform_device_register(&tncommon_smsc911x_device);
}

static int __init tncommon_smsc911x_eth_addr_setup(char *str)
{
	int i;

	if(str == NULL)
		return 0;
	for(i = 0; i <  ETH_ALEN; i++)
		tncommon_smsc911x_config.mac[i] = simple_strtol(&str[i*3],
								(char **)NULL, 16);
	return 1;
}
__setup("eth1addr=", tncommon_smsc911x_eth_addr_setup);

#else
static inline void __init tncommon_smsc911x_init(void) { return; }
#endif

int tncommon_start_external_netdevs(struct tncommon_smsc911x_platform_control *smsc_chip)
{
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	int irq = smsc_chip->irq_gpio;
	int reset = smsc_chip->reset_gpio;
	int cs = smsc_chip->gpmc_cs_number;
	printk("SMSC9220 Board Init for %s\n", smsc_chip[0].name);
	if (irq < 0 || reset < 0 || cs < 0) { printk("No SMSC9220 Present!!\n"); return 1; }

	printk("SMSC Resource: irq %d, reset %d, cs %d\n",irq,reset,cs);
	tncommon_smsc911x_gpio_irq = irq;
	tncommon_smsc911x_gpio_reset = reset;
	tncommon_smsc911x_cs = cs;

	tncommon_smsc911x_init();
#endif
	return 0;
}

/****************************************************************************
 *
 * OMAP2+  DSS - GPIO Control
 *
 ****************************************************************************/

/*GPIO_0 is exclude out from dss gpio, since it is often system crutial control*/

static struct tncommon_gpio_control_type *tncommon_dss_gpio = NULL;

#define TNCOMMON_MAX_INTERNAL_GPIO_NUMBER	191

static void tncommon_dss_setup_gpios(int onoff)
{
	int gpio;
	int enable_type;
	int i;
	for(i=0;i<TNCOMMON_MAX_DSS_CONTROL_GPIO;i++)
	{
		gpio = tncommon_dss_gpio[i].gpio;
		enable_type = tncommon_dss_gpio[i].enable_type;
		if(gpio<=0 || enable_type<0)
			continue;

		if(enable_type)
		{
			printk("=-=-=-=-=-=-tncommon_dss_setup_gpio %d val %d\n",gpio,onoff);
			if(gpio>TNCOMMON_MAX_INTERNAL_GPIO_NUMBER)
				gpio_set_value_cansleep(gpio,onoff);
			else
				gpio_set_value(gpio,onoff);

		}
		else
		{
			printk("=-=-=-=-=-=-tncommon_dss_setup_gpio %d val %d\n",gpio,!onoff);
			if(gpio>TNCOMMON_MAX_INTERNAL_GPIO_NUMBER)
				gpio_set_value_cansleep(gpio, !onoff);			
			else
				gpio_set_value(gpio, !onoff);

		}
	}
}

static void __init tncommon_dss_request_gpio(struct tncommon_gpio_control_type *gpio_type)
{
	int gpio = gpio_type->gpio;
	int enable_type = gpio_type->enable_type;

	printk("=-=-=-=-=-=-tncommon_dss_request_gpio %d\n",gpio);

	if(gpio <= 0)
		return;

	gpio_request(gpio,"");
	gpio_export(gpio, 0);

/*Bootload FB Splash enabled, default ON*/
#if defined(CONFIG_FB_OMAP_BOOTLOADER_INIT)
	gpio_direction_output(gpio, enable_type); 
#else
	gpio_direction_output(gpio, !enable_type); 
#endif

	return;
}

/****************************************************************************
 *
 * OMAP2+  DSS - Backlight Control
 *
 ****************************************************************************/

#define TNCOMMON_DEFAULT_BACKLIGHT_LEVEL 100

#include <linux/backlight.h>
#include <linux/i2c/twl.h>

/*tncommon_backlight_level store last status of brightness level.
It should be always greater than 0 to exclude enable/disable panel case*/
static int tncommon_backlight_level=TNCOMMON_DEFAULT_BACKLIGHT_LEVEL;

static void (*tncommon_backlight_set_intensity) (int);

static void tncommon_dummy_backlight_set_intensity(int intensity) {
	if(intensity > 0)
	        tncommon_backlight_level=intensity;
        printk("dummy bklight device: set intensity %d \n",intensity);
}

static char * bklight_device_name=NULL;

static struct generic_bl_info tncommon_bklight_platform_data = {
	.name			= "generic-bklight",
	.max_intensity		= 100,
	.default_intensity	= TNCOMMON_DEFAULT_BACKLIGHT_LEVEL,
	.limit_mask		= 0,
	.set_bl_intensity	= tncommon_dummy_backlight_set_intensity,
	.kick_battery		= NULL,
};

static struct platform_device tncommon_backlight_devices = {
	.name		= "generic-bl",
	.id		= -1,
	.dev		= {
		.platform_data	= &tncommon_bklight_platform_data,
	},
};

#if defined(CONFIG_TWL4030_CORE)

#define TWL_INTBR_GPBR1 0x0c
#define TWL_INTBR_PMBR1 0x0d
#define TWL_PWM_ON    	0x00
#define TWL_PWM_OFF  	0x01

static void tncommon_twlpwm_backlight_set_intensity(int intensity)
{
	unsigned char backlight_intensity;

	if(intensity > 0)
		tncommon_backlight_level = intensity;

	printk("bkligt_device:%s:set intensity %d\n",(bklight_device_name!=NULL)?bklight_device_name:"",intensity);

	/*
	 * For 0,1%   of duty cycle (PWM_ON, PWM_OFF) = (0x0f, 0x10)
	 * For 11 %   of duty cycle (PWM_ON, PWM_OFF) = (0x01, 0x10)
	 * For 48%    of duty cycle (PWM_ON, PWM_OFF) = (0x01, 0x3f)
	 * For 73.4%  of duty cycle (PWM_ON, PWM_OFF) = (0x01, 0x5f)
	 * For 85.92% of duty cycle (PWM_ON, PWM_OFF) = (0x01, 0x6f)
	 * For 98%    of duty cycle (PWM_ON, PWM_OFF) = (0x01, 0x7f)
	*/

	/*Character of Other LCD Interface control:
	 *PWM0/TWL_GPIO6 set as PWM to control backlight Dimming
	 *PWM1/TWL_GPIO7 reserve for PWM controlled LED power source*/

	//PWM0 Enable set to 0x04, Enable PWM1 set to 0x30, Both set to 0x34
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x04, TWL_INTBR_PMBR1);
	//PWM0 Enable set to 0x05, PWM1 Enable set to 0x0a, Both set to 0x0f
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x05, TWL_INTBR_GPBR1);

	if(intensity == 0)
		backlight_intensity = 0;/*For certain panel, they require PWM set to 0 to turn off*/
	else
		backlight_intensity = ((125 * (intensity)) / 100) + 2;
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, 0x1, TWL_PWM_ON);
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, backlight_intensity , TWL_PWM_OFF);
}
#else
static inline void tncommon_twlpwm_backlight_set_intensity(int intensity) { return; }
#endif

static struct tncommon_gpio_control_type bklight_gpio;
static void tncommon_gpio_backlight_onoff(int intensity)
{
	int gpio;
	int enable_type;

	gpio = bklight_gpio.gpio;
	enable_type = bklight_gpio.enable_type;

	/*This check valid of control, both gpio and enable_type should larger then 0 */
	if(gpio<0 || enable_type<0)
		return;

	printk("bkligt_device:%s:set intensity %d\n",(bklight_device_name!=NULL)?bklight_device_name:"",intensity);

	if(intensity > 0)
		tncommon_backlight_level = intensity;

	if(intensity > 0)
		gpio_set_value(gpio,enable_type);
	else
		gpio_set_value(gpio, !enable_type);

}

#if defined(CONFIG_OMAP_GPTIMER9_PWM)
extern int gptimer9_pwm_init(void);
extern int gptimer9_pwm_set_duty_cycle(u32 duty_cycle);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux tncommon_gpt9pwm_bklihgt_mux[] __initdata = {
	OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE2 | OMAP_PULL_UP), /* BL */
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static void tncommon_gpt9_pwm_evt_set_intensity(int intensity)
{
	int ret=-1;
	
	if(intensity <= 0)
		intensity = 0;

	if(intensity > 0)
		tncommon_backlight_level = intensity;

	printk("bkligt_device:%s:set intensity %d\n",(bklight_device_name!=NULL)?bklight_device_name:"",intensity);

	ret = gptimer9_pwm_set_duty_cycle(intensity);
	if(ret != 0)
		printk("%s: Error setting duty cycle\n",__FUNCTION__);
}
#endif

static int tncommon_register_backlight_devices(struct tncommon_backlight_control *bklight)
{

	if(!strcmp("gpio_backlight",bklight->name))
	{
		bklight_device_name = bklight->name;
		tncommon_dss_request_gpio(&bklight->pwr_gpio);
		bklight_gpio.gpio = bklight->pwr_gpio.gpio;
		bklight_gpio.enable_type = bklight->pwr_gpio.enable_type;
		tncommon_backlight_set_intensity = &tncommon_gpio_backlight_onoff;
	}
	else if(!strcmp("gpt9_pwm_evt",bklight->name))
	{
	#if defined(CONFIG_OMAP_GPTIMER9_PWM)
		bklight_device_name = bklight->name;
		tncommon_backlight_set_intensity = &tncommon_gpt9_pwm_evt_set_intensity;
		if(!gptimer9_pwm_init())
		{
			tncommon_gpt9_pwm_evt_set_intensity(100);
			omap3_mux_init(tncommon_gpt9pwm_bklihgt_mux, OMAP_PACKAGE_CBB);	
		}
	#else
		printk("%s:No target device, set to dummy.\n",__FUNCTION__);
		tncommon_backlight_set_intensity = &tncommon_dummy_backlight_set_intensity;		
	#endif
	}
	else if(!strcmp("twlpwm-bklight",bklight->name))
	{
	#if defined(CONFIG_TWL4030_CORE)
		bklight_device_name = bklight->name;
		tncommon_backlight_set_intensity = &tncommon_twlpwm_backlight_set_intensity;
	#else
		printk("%s:No target device, set to dummy.\n",__FUNCTION__);
		tncommon_backlight_set_intensity = &tncommon_dummy_backlight_set_intensity;		
	#endif
	}
	else
		tncommon_backlight_set_intensity = &tncommon_dummy_backlight_set_intensity;

	if(tncommon_backlight_set_intensity != NULL)
		tncommon_bklight_platform_data.set_bl_intensity = tncommon_backlight_set_intensity;

	platform_device_register(&tncommon_backlight_devices);

	return 0;
}

/****************************************************************************
 *
 * OMAP2+  DSS - Display Subsystem
 *
 ****************************************************************************/

static int tncommon_dss_enable_panel(struct omap_dss_device *dssdev)
{
	tncommon_dss_setup_gpios(1);
	if(tncommon_backlight_set_intensity != NULL)
		tncommon_backlight_set_intensity(TNCOMMON_DEFAULT_BACKLIGHT_LEVEL/*tncommon_backlight_level*/);
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 400000);
	return 0;
}

static void tncommon_dss_disable_panel(struct omap_dss_device *dssdev)
{
	tncommon_dss_setup_gpios(0);
	if(tncommon_backlight_set_intensity != NULL)
		tncommon_backlight_set_intensity(0);
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

static struct omap_dss_device tncommon_dss_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.reset_gpio = -EINVAL,
	.platform_enable	= tncommon_dss_enable_panel,
	.platform_disable	= tncommon_dss_disable_panel,
};


static struct omap_dss_device tncommon_dss_tv_device = {
	.name 			= "tv",
	.driver_name 		= "venc",
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type	= OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable	= tncommon_dss_enable_panel,
	.platform_disable	= tncommon_dss_disable_panel,
};

static struct omap_dss_device tncommon_dss_dpi_device = {
	.name			= "dpi",
	.driver_name		= "tnlcd",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= tncommon_dss_enable_panel,
	.platform_disable	= tncommon_dss_disable_panel,
};

static struct omap_dss_device *tncommon_dss_devices[] = {
	&tncommon_dss_dpi_device,
	&tncommon_dss_tv_device,
	&tncommon_dss_hdmi_device,
};

static struct omap_dss_board_info tncommon_dss_data = {
	/*omap/am/dm/35xx/37xx has 2 phisical video output*/
	.num_devices		= 2,
	.devices		= tncommon_dss_devices,
	.default_device	= &tncommon_dss_dpi_device,
};

struct platform_device tncommon_dss_platform_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &tncommon_dss_data,
	},
};

/*Temporally fix for TDM3730 with Sil9022*/
void tncommon_enable_hdmi(void ) { tncommon_dss_data.num_devices = 3; }

int __init tncommon_display_subsystem_init(struct tncommon_dss_panel_control *dss_controls)
{
	int i;
	tncommon_backlight_set_intensity = &tncommon_dummy_backlight_set_intensity;

	tncommon_dss_gpio = dss_controls->gpios;
 
	for(i=0; i<TNCOMMON_MAX_DSS_CONTROL_GPIO ; i++ )
		tncommon_dss_request_gpio(&tncommon_dss_gpio[i]);

	platform_device_register(&tncommon_dss_platform_device);

	tncommon_register_backlight_devices(&dss_controls->bklight);

	return 0;
}
/*******************************************
* WIFI POWER
********************************************/
static int tncommon_gpio_wifi_power=-1;
void tncommon_wifi_device_power_switch(int onoff)
{
	if(tncommon_gpio_wifi_power<0)
		return;

	if(onoff)
		gpio_set_value(tncommon_gpio_wifi_power, 0);
	else
		gpio_set_value(tncommon_gpio_wifi_power, 1);
}
EXPORT_SYMBOL(tncommon_wifi_device_power_switch);

void tncommon_mmc2_wifi_init(void)
{
	if (cpu_is_omap3505() || cpu_is_omap3517())
		tncommon_gpio_wifi_power = 156;
	else
		tncommon_gpio_wifi_power = 157;
	if ((gpio_request(tncommon_gpio_wifi_power, "GPIO_WIFI_POWER") == 0) &&
            (gpio_direction_output(tncommon_gpio_wifi_power, 1) == 0)) {
                gpio_export(tncommon_gpio_wifi_power, 0);
#if defined(CONFIG_MMC_OMAP_HS_SDIO_SWITCH)
		/*Default Off*/
		tncommon_wifi_device_power_switch(0);
#else
		/*Default On*/
		tncommon_wifi_device_power_switch(1);
#endif
        }else
                pr_warning("TNCOMMON : Could not obtain gpio GPIO_WIFI_POWER\n");
}

/*****************************************************
Board Identication and Setup 
*****************************************************/

int tncommon_board_type = TNCOMMON_BOARD_TYPE_INVALID;

/*
0:blizzard
1:twister
2:tsunami
3:thunder
4:inferno
5:blizzard_a2
6:thb
*/

int __init tncommon_setup_board_type(char *str)
{
	/*I need something more to identify the board*/
	if(!strcmp(str, "blizzard"))
		tncommon_board_type = TNCOMMON_BOARD_TYPE_BLIZZARD;
	else if(!strcmp(str, "twister"))
		tncommon_board_type = TNCOMMON_BOARD_TYPE_TWISTER ;
	else if(!strcmp(str, "tsunami"))
		tncommon_board_type = TNCOMMON_BOARD_TYPE_TSUNAMI ;
	else if(!strcmp(str, "thunder"))
		tncommon_board_type = TNCOMMON_BOARD_TYPE_THUNDER ;
	else if(!strcmp(str, "inferno"))
		tncommon_board_type = TNCOMMON_BOARD_TYPE_INFERNO ;
	else if(!strcmp(str, "blizzard_a2"))
		tncommon_board_type = TNCOMMON_BOARD_TYPE_BLIZZARD_A2 ;
	else if(!strcmp(str, "thb"))
		tncommon_board_type = TNCOMMON_BOARD_TYPE_THB ;
	else
		tncommon_board_type = TNCOMMON_BOARD_TYPE_INVALID;

	return 0;
}

struct tncommon_smsc911x_platform_control tncommon_smsc911x_settting[] = 
{
	{ "blizzard"	,1	,9	, 5},
	{ "twister"	,153	,142	, 5},
	{ "tsunami"	,143	,142	, 5},
	{ "thunder"	,-1	,-1	,-1},
	{ "inferno"	,-1	,-1	,-1},
	{ "blizzard_a2"	,159	,186	, 5},
	{ "thb"		,-1	,-1	,-1},
	{ "invalid"	,-1	,-1	,-1}
};

struct tncommon_dss_panel_control dss_controls[] = 
{	/*Don't put IO-EX and PMIC gpios here, it will hang*/
	{"blizzard",{{138,1},{139,1},{-1,-1},{-1,-1}},{"twlpwm-bklight",-1,{-1,-1}}},
#if defined(CONFIG_OMAP_GPTIMER9_PWM)
	{"twister",{{138,0},{139,1},{24,1},{-1,-1}},{"gpt9_pwm_evt",-1,{-1,-1}}},
#else
	{"twister",{{138,0},{139,1},{24,1},{-1,-1}},{"gpio_backlight",1,{53,1}}},
#endif
	{"tsunami",{{138,0},{139,1},{-1,-1},{-1,-1}},{"twlpwm-bklight",-1,{-1,-1}}},
	{"thunder",{{138,0},{139,1},{-1,-1},{-1,-1}},{"twlpwm-bklight",-1,{-1,-1}}},
	{"inferno",{{138,0},{139,1},{-1,-1},{-1,-1}},{"",-1,{-1,-1}}},
	{"blizzard_a2",{{138,0},{139,1},{-1,-1},{-1,-1}},{"twlpwm-bklight",-1,{-1,-1}}},
#if defined(CONFIG_OMAP_GPTIMER9_PWM)
	{"thb",{{138,0},{139,1},{24,1},{-1,-1}},{"gpt9_pwm_evt",-1,{-1,-1}}},
#else
	{"thb",{{138,0},{139,1},{24,1},{-1,-1}},{"gpio_backlight",1,{53,1}}},
#endif
	{"invalid",{{-1,-1},{-1,-1},{-1,-1}},{"",-1,{-1,-1}}}
};


