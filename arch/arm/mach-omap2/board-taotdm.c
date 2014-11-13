/*
 * TechNexion TAO3530/TDM3730
 *
 * Copyright (C) 2012 TechNexion Inc
 *
 * Maintainer: TechNexion  <linuxfae@technexion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mmc/host.h>

#include <plat/omap-pm.h>
#include <plat/usb.h>

#include <plat/board.h>
#include <plat/common.h>

#include <linux/regulator/machine.h>

#ifdef CONFIG_PANEL_SIL9022
#include <linux/sil9022.h>
#endif

#include <plat/gpmc.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/atmel_mxt_ts.h>

#include "sdram-micron-mt46h32m32lf-6.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"
#include "sdram-hynix-h8kds0un0mer-4em.h"

#include "mux.h"
#include "hsmmc.h"
#include "control.h"
#include "timer-gp.h"

#include <linux/usb/otg.h>
#include <mach/hardware.h>

#include "board-technexion-common.h"

//#define USE_OHCI_USB
#undef USE_OHCI_USB

/****************************************************************************
 *
 * OMAP3 Power Manager
 *
 ****************************************************************************/

extern void omap_pm_sys_offmode_select(int);
extern void omap_pm_sys_offmode_pol(int);
extern void omap_pm_sys_clkreq_pol(int);
extern void omap_pm_auto_off(int);
extern void omap_pm_auto_ret(int);

/**
 * Board specific initialization of PM components
 */
static void __init taotdm_pm_init(void)
{
	/* Use sys_offmode signal */
	omap_pm_sys_offmode_select(1);

	/* sys_clkreq - active high */
	omap_pm_sys_clkreq_pol(1);

	/* sys_offmode - active low */
	omap_pm_sys_offmode_pol(0);

	/* Automatically send OFF command */
	omap_pm_auto_off(1);

	/* Automatically send RET command */
	omap_pm_auto_ret(1);
}

/****************************************************************************
 *
 * TPS65930 voltage regulator
 *
 ****************************************************************************/

static struct regulator_consumer_supply taotdm_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply taotdm_vmmc2_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply taotdm_vsim_supply = {
	.supply			= "vmmc_aux",
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data taotdm_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &taotdm_vmmc1_supply,
};

/* VMMC2 */
static struct regulator_init_data taotdm_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &taotdm_vmmc2_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data taotdm_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &taotdm_vsim_supply,
};

static struct regulator_consumer_supply taotdm_vaux2_supply = {
	.supply		= "cam_io_1v8",
};


/* VAUX2 for CAM_IO_1V8 */
static struct regulator_init_data taotdm_vaux2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &taotdm_vaux2_supply,
};

static struct regulator_consumer_supply taotdm_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss");


/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data taotdm_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &taotdm_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply taotdm_vpll2_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");


static struct regulator_init_data taotdm_vpll2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &taotdm_vpll2_supply,
};

#if defined(CONFIG_TWL4030_SCRIPT)
/****************************************************************************
 *
 * TPS65930 Power Contorl Script
 *
 ****************************************************************************/

/**
 * Macro to configure resources
 */
#define TWL4030_RESCONFIG(res,grp,typ1,typ2,state)	\
	{						\
		.resource	= res,			\
		.devgroup	= grp,			\
		.type		= typ1,			\
		.type2		= typ2,			\
		.remap_sleep	= state			\
	}

static struct twl4030_resconfig  __initdata taotdm_board_twl4030_rconfig[] = {
	TWL4030_RESCONFIG(RES_VPLL1, DEV_GRP_P1, 3, 1, RES_STATE_OFF),		/* ? */
	TWL4030_RESCONFIG(RES_VINTANA1, DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VINTANA2, DEV_GRP_ALL, 0, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VINTDIG, DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VIO, DEV_GRP_ALL, 2, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VDD1, DEV_GRP_P1, 4, 1, RES_STATE_OFF),		/* ? */
	TWL4030_RESCONFIG(RES_VDD2, DEV_GRP_P1, 3, 1, RES_STATE_OFF),		/* ? */
	TWL4030_RESCONFIG(RES_REGEN, DEV_GRP_ALL, 2, 1, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_NRES_PWRON, DEV_GRP_ALL, 0, 1, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_CLKEN, DEV_GRP_ALL, 3, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_SYSEN, DEV_GRP_ALL, 6, 1, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_HFCLKOUT, DEV_GRP_P3, 0, 2, RES_STATE_SLEEP),	/* ? */
	TWL4030_RESCONFIG(0, 0, 0, 0, 0),
};

/**
 * Optimized 'Active to Sleep' sequence
 */
static struct twl4030_ins taotdm_sleep_seq[] __initdata = {
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_SLEEP), 20},
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_SLEEP), 2 },
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_SLEEP), 2 },
};

static struct twl4030_script taotdm_sleep_script __initdata = {
	.script	= taotdm_sleep_seq,
	.size	= ARRAY_SIZE(taotdm_sleep_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/**
 * Optimized 'Sleep to Active (P12)' sequence
 */
static struct twl4030_ins taotdm_wake_p12_seq[] __initdata = {
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script taotdm_wake_p12_script __initdata = {
	.script = taotdm_wake_p12_seq,
	.size   = ARRAY_SIZE(taotdm_wake_p12_seq),
	.flags  = TWL4030_WAKEUP12_SCRIPT,
};

/**
 * Optimized 'Sleep to Active' (P3) sequence
 */
static struct twl4030_ins taotdm_wake_p3_seq[] __initdata = {
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script taotdm_wake_p3_script __initdata = {
	.script = taotdm_wake_p3_seq,
	.size   = ARRAY_SIZE(taotdm_wake_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

/**
 * Optimized warm reset sequence (for less power surge)
 */
static struct twl4030_ins taotdm_wrst_seq[] __initdata = {
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2 },
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_WRST), 0x2},
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VPLL1, RES_STATE_WRST), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VDD2, RES_STATE_WRST), 0x7 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VDD1, RES_STATE_WRST), 0x25 },
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_WRST), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 0x2 },
};

static struct twl4030_script taotdm_wrst_script __initdata = {
	.script = taotdm_wrst_seq,
	.size   = ARRAY_SIZE(taotdm_wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script __initdata *board_twl4030_scripts[] = {
	&taotdm_wake_p12_script,
	&taotdm_wake_p3_script,
	/* Currently, while reset, somehow enable_off will also trigger, which 
	   system will enter sleep mode instead of reset, temporally remove 
	   sleep script, the solution could be modifying trigger of sleep to
	   multiple signal to avoid this.
	&taotdm_sleep_script,
	*/
	&taotdm_wrst_script
};

static struct twl4030_power_data __initdata taotdm_script_data = {
	.scripts		= board_twl4030_scripts,
	.num			= ARRAY_SIZE(board_twl4030_scripts),
	.resource_config	= taotdm_board_twl4030_rconfig,
};
#endif

/****************************************************************************
 *
 *  GPIO_LED
 *
 ****************************************************************************/
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led taotdm_gpio_leds[] = {
	{
		.name			= "taotdm::pmu_stat",
		.default_trigger	= "mmc0",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data taotdm_gpio_led_info = {
	.leds		= taotdm_gpio_leds,
	.num_leds	= ARRAY_SIZE(taotdm_gpio_leds),
};

static struct platform_device taotdm_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &taotdm_gpio_led_info,
	},
};

static void __init taotdm_status_led_init(void)
{
	platform_device_register(&taotdm_leds_gpio);
}
#else
static inline void __init taotdm_status_led_init(void) { return; }
#endif

/****************************************************************************
 *
 * GPIO_BUTTON
 *
 ****************************************************************************/
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button taotdm_gpio_buttons[] = {
        {
                .code                   = KEY_ESC,
                .gpio                   = 7,
                .desc                   = "Back",
                .wakeup                 = 1,
        },
};

static struct gpio_keys_platform_data taotdm_gpio_key_info = {
        .buttons        = taotdm_gpio_buttons,
        .nbuttons       = ARRAY_SIZE(taotdm_gpio_buttons),
};

static struct platform_device taotdm_keys_gpio = {
        .name   = "gpio-keys",
        .id     = -1,
        .dev    = {
                .platform_data  = &taotdm_gpio_key_info,
        },
};

static void __init taotdm_gpio_keys_init(void)
{
	platform_device_register(&taotdm_keys_gpio);
}
#else
static inline void __init taotdm_gpio_keys_init(void) { return; }
#endif

/****************************************************************************
 *
 * MMC bus
 *
 ****************************************************************************/

static struct omap2_hsmmc_info taotdm_mmc[] = {
	//SDcard
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	//Wifi
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.transceiver	= false,
		.ext_clock	= false,
	},
	{}	/* Terminator */
};


/****************************************************************************
 *
 * TPS65930 Devices : GPIO, CODEC, ULPI
 *
 ****************************************************************************/
static int taotdm_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	taotdm_mmc[0].gpio_wp = -EINVAL;
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	taotdm_mmc[0].gpio_cd = gpio + 0;
#if !defined(CONFIG_LIBERTAS_SDIO_MODULE)
	/*This enables WIFI_CD, It will change with WIFI_POWER*/
	omap_mux_init_gpio(54, OMAP_PIN_INPUT);
	taotdm_mmc[1].gpio_cd = 54;
#endif
	omap2_hsmmc_init(taotdm_mmc);

	/* link regulators to MMC adapters */
	taotdm_vmmc1_supply.dev = taotdm_mmc[0].dev;
	taotdm_vmmc2_supply.dev = taotdm_mmc[1].dev;
	taotdm_vsim_supply.dev = taotdm_mmc[0].dev;

	/* REVISIT: need ehci-omap hooks for external VBUS
	 * power switch and overcurrent detect
	 */
	gpio_request(gpio + 1, "EHCI_nOC");
	gpio_direction_input(gpio + 1);

	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	taotdm_gpio_leds[0].gpio = gpio + TWL4030_GPIO_MAX + 1;
#endif

	/* gpio + 7 == DVI Enable */
	gpio_request(gpio + 7, "EN_DVI");
	gpio_export(gpio + 7, 0);/*Export it for usespace*/
	gpio_direction_output(gpio + 7, 0);

	taotdm_status_led_init();
	return 0;
}

static struct twl4030_gpio_platform_data taotdm_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= taotdm_twl_gpio_setup,
};

static struct twl4030_usb_data taotdm_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_madc_platform_data taotdm_madc_data = {
	.irq_line	= 1,
};

/*AUDIO CODEC*/
static struct twl4030_codec_audio_data taotdm_audio_data = {
	.audio_mclk = 26000000,
	.digimic_delay = 1,
	.ramp_delay_value = 1,
	.offset_cncl_path = 1,
	.check_defaults = false,
	.reset_registers = false,
	.reset_registers = false,
};

static struct twl4030_codec_data taotdm_codec_data = {
	.audio_mclk = 26000000,
	.audio = &taotdm_audio_data,
};

static struct twl4030_platform_data taotdm_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &taotdm_madc_data,
	.usb		= &taotdm_usb_data,
	.gpio		= &taotdm_gpio_data,
	.codec		= &taotdm_codec_data,
	.vmmc1		= &taotdm_vmmc1,
	.vmmc2		= &taotdm_vmmc2,
	.vsim		= &taotdm_vsim,
	.vdac		= &taotdm_vdac,
	.vpll2		= &taotdm_vpll2,
	.vaux2		= &taotdm_vaux2,
#if 0/*defined(CONFIG_TWL4030_SCRIPT)*/
	.power		= &taotdm_script_data,
#endif
};

/****************************************************************************
 *
 * I2C
 *
 ****************************************************************************/

static struct i2c_board_info __initdata taotdm_i2c_1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &taotdm_twldata,
	},
};

#define BLIZZARD_PRISM_GPIO_TS_RESET 	14
#define BLIZZARD_PRISM_GPIO_TS_IRQ 	15
#define GPIO_TS_ATMEL_IRQ		136
//#define GPIO_TS_ATMEL_IRQ		106

static u8 read_irq_level(void)
{
    //printk("read_irq_level called\n");
    return gpio_get_value(GPIO_TS_ATMEL_IRQ) ? 0 : 1;
}

static struct mxt_platform_data atmel_ts_pdata = {
    .irqflags 	= IRQF_TRIGGER_FALLING,
    .read_chg = &read_irq_level,
    .input_name = "Atmel Multitouch",
};

/*
static struct mxt_platform_data atmel_ts_pdata = {
    .irqflags 	= IRQF_TRIGGER_FALLING,
    //.read_chg = &read_irq_level,
    //.input_name = "Atmel Multitouch",
    .x_line	= 24,
    .y_line	= 14,
    .x_size	= 4096,
    .y_size	= 4096,
    .blen	= 0x21,
    .threshold	= 0x28,
    .voltage	= 3000000,
    .orient	= MXT_DIAGONAL,
};
*/
static struct i2c_board_info __initdata taotdm_i2c_2_boardinfo[] = {
#if defined(CONFIG_TOUCHSCREEN_PRISM)
        {
                I2C_BOARD_INFO("prism", 0x10),
                .irq = OMAP_GPIO_IRQ(BLIZZARD_PRISM_GPIO_TS_IRQ),
                .flags = I2C_CLIENT_WAKE,
        },
#endif
//#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT
	{
		I2C_BOARD_INFO("atmel_mxt_ts",0x4D),
		.irq = OMAP_GPIO_IRQ(GPIO_TS_ATMEL_IRQ),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &atmel_ts_pdata,
	},
//#endif
};

static struct i2c_board_info __initdata taotdm_i2c_3_boardinfo[] = {
#if defined(CONFIG_RTC_DRV_DS1307) || \
        defined(CONFIG_RTC_DRV_DS1307_MODULE)
        {
                I2C_BOARD_INFO("ds1337", 0x68),
        },
#endif
/*
	{
		I2C_BOARD_INFO("atmel_mxt_ts",0x4D),
		.irq = OMAP_GPIO_IRQ(GPIO_TS_ATMEL_IRQ),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &atmel_ts_pdata,
	},
*/
	{
		I2C_BOARD_INFO("stc3105iqt",0x70),
	},
};

static int __init taotdm_i2c_init(void)
{
	printk("taotdm i2c init\n");
#ifdef CONFIG_TOUCHSCREEN_PRISM
        if (!gpio_request(BLIZZARD_PRISM_GPIO_TS_RESET, "ts reset") && !gpio_direction_output(BLIZZARD_PRISM_GPIO_TS_RESET, 0)) {
                printk(" Blizzard prism: resetting touchscreen\n");
                gpio_export(BLIZZARD_PRISM_GPIO_TS_RESET, 0);
                gpio_set_value(BLIZZARD_PRISM_GPIO_TS_RESET, 0);
                mdelay(50);
                gpio_set_value(BLIZZARD_PRISM_GPIO_TS_RESET, 1);
        } else
                printk("GPIO 14 could not be reserved or set as output\n");

        if (gpio_request(BLIZZARD_PRISM_GPIO_TS_IRQ, "ts irq") || gpio_direction_input(BLIZZARD_PRISM_GPIO_TS_IRQ)) {
                printk("GPIO 15 could not be reserved or set as input\n");
                gpio_export(BLIZZARD_PRISM_GPIO_TS_IRQ, 0);
        }
#endif
//#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT
        if (gpio_request(GPIO_TS_ATMEL_IRQ, "ts irq") || gpio_direction_input(GPIO_TS_ATMEL_IRQ)) {
                printk("GPIO 136 could not be reserved or set as input\n");
        } else {
    		printk("Requested Atmel gpio irq\n");
                gpio_export(GPIO_TS_ATMEL_IRQ, 0);
                gpio_set_debounce(GPIO_TS_ATMEL_IRQ, 0xa);
        }
//#endif

	omap_register_i2c_bus(1, 2600, taotdm_i2c_1_boardinfo, ARRAY_SIZE(taotdm_i2c_1_boardinfo));
	omap_register_i2c_bus(2,  400, taotdm_i2c_2_boardinfo, ARRAY_SIZE(taotdm_i2c_2_boardinfo));
	omap_register_i2c_bus(3,  400, taotdm_i2c_3_boardinfo, ARRAY_SIZE(taotdm_i2c_3_boardinfo));
	return 0;
}

/****************************************************************************
 *
 * USB
 *
 ****************************************************************************/

static struct omap_musb_board_data taotdm_musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
/*	.power			= 100,	*/
	.power 		= 500,
/*	.extvbus 		= 1,	*/
};

#define TAOTDM_EHCI_RESET	162

#ifdef USE_OHCI_USB
static struct ohci_hcd_omap_platform_data ohci_pdata __initconst = {
	.port_mode[0] = OMAP_OHCI_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_TLL_2PIN_DPDM,
	//.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.port_mode[2] = OMAP_OHCI_PORT_MODE_UNUSED,
	//.port_mode[0] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	//.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	//.port_mode[2] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.es2_compatibility = false
	//.port_mode[3] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM
};

static __init void taotdm_ehci_init(void)
{
    usb_ohci_init(&ohci_pdata);
}

#else
static const struct ehci_hcd_omap_platform_data taotdm_ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = TAOTDM_EHCI_RESET,
	.reset_gpio_port[2]  = -EINVAL
};

static __init void taotdm_ehci_init(void)
{
	/* Configure GPIO for EHCI port */
	
	omap_mux_init_gpio(TAOTDM_EHCI_RESET, OMAP_PIN_OUTPUT);

	gpio_request(TAOTDM_EHCI_RESET, "USB_RESET");
	gpio_direction_output(TAOTDM_EHCI_RESET, 0);
	gpio_export(TAOTDM_EHCI_RESET, 0);
	msleep(100);
	gpio_set_value(TAOTDM_EHCI_RESET, 1);
        
	usb_ehci_init(&taotdm_ehci_pdata);
}
#endif

/****************************************************************************
 *
 * MUX
 *
 ****************************************************************************/

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux tdm3730_board_mux[] __initdata = {
        /*
        OMAP3_MUX(CAM_D0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D8, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D9, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D10, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D11, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        */
        /* Camera - HS/VS signals */
        //OMAP3_MUX(CAM_HS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        
        //HDMI PD, 0 to powerdown
        OMAP3_MUX(CAM_VS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	//beeper
	OMAP3_MUX(ETK_CTL, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	//backlight
	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	
        /* Camera - Reset GPIO 98 */
        //OMAP3_MUX(CAM_FLD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

        /* Camera - XCLK */
        //OMAP3_MUX(CAM_XCLKA, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP | 
				OMAP_PIN_OFF_WAKEUPENABLE), /* Touchscreen irq */
        /*
	OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | 
				OMAP_PIN_OFF_WAKEUPENABLE),
        */
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux tao3530_board_mux[] __initdata = {
        /* Camera - Parallel Data */
        OMAP3_MUX(CAM_D0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D8, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D9, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D10, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_D11, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

        /* Camera - HS/VS signals */
        OMAP3_MUX(CAM_HS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(CAM_VS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

        /* Camera - Reset GPIO 98 */
        OMAP3_MUX(CAM_FLD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

        /* Camera - XCLK */
        OMAP3_MUX(CAM_XCLKA, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(ETK_D8, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP | 
				OMAP_PIN_OFF_WAKEUPENABLE), /* Touchscreen irq */

	OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT | 
				OMAP_PIN_OFF_WAKEUPENABLE),

        { .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif

/****************************************************************************
 *
 * Platform
 *
 ****************************************************************************/


static struct omap_board_config_kernel taotdm_board_config[] __initdata = {
};

static void __init taotdm_init_irq(void)
{
	omap2_init_common_infrastructure();
	if (cpu_is_omap3630())
        omap2_init_common_devices(h8kds0un0mer4em_sdrc_params,
                                  h8kds0un0mer4em_sdrc_params);
	else
        omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
                                  mt46h32m32lf6_sdrc_params);

	omap_init_irq();
	gpmc_init();
#ifdef CONFIG_OMAP_32K_TIMER
	if (cpu_is_omap3630())
		omap2_gp_clockevent_set_gptimer(1);
	else
		omap2_gp_clockevent_set_gptimer(12);
#endif
}


static struct platform_device *taotdm_devices[] __initdata = {
};

static int __init taotdm_setup_board_devices(void)
{
	/*This is write to init nand,
	  we should be able init something else as well.*/
	tncommon_flash_devices_init();

	taotdm_ehci_init();

	tncommon_display_subsystem_init(&dss_controls[tncommon_board_type]);

	tncommon_start_external_netdevs(&tncommon_smsc911x_settting[tncommon_board_type]);
 
	tncommon_start_tsc_devices();

	taotdm_gpio_keys_init();

#ifdef CONFIG_USB_ANDROID
	tncommon_android_gadget_init();
#endif
	return 0;
}


static int __init taotdm_setup_board_type(char *str)
{
	return tncommon_setup_board_type(str);
}
__setup("boardname=", taotdm_setup_board_type);


static int __init taotdm_enable_hdmi_support(char *str)
{
	
	if (cpu_is_omap3630())
		tncommon_enable_hdmi();

        return 0;
}
__setup("enablehdmi", taotdm_enable_hdmi_support);


static void __init tdm3730_init(void)
{
#ifdef CONFIG_OMAP_MUX
	omap3_mux_init(tdm3730_board_mux, OMAP_PACKAGE_CBB);
#endif

	/*We put something here to add devices into device list, 
	  so i2c_init and other init can initial those
	  , we use taotdm_setup_board_type() to do this job....*/

#if defined(CONFIG_TWL4030_SCRIPT)
        taotdm_twldata.power = &taotdm_script_data;
#endif

	taotdm_i2c_init();

	tncommon_mmc2_wifi_init();

	omap_serial_init();

	usb_musb_init(&taotdm_musb_board_data);

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	/*Call require function to register plat-dev 1-by-1*/
	if(tncommon_board_type==TNCOMMON_BOARD_TYPE_INVALID)
		tncommon_board_type = TNCOMMON_BOARD_TYPE_BLIZZARD;
	taotdm_setup_board_devices();

	taotdm_pm_init();
}

/* Maintainer: TechNexion: linuxfae@technexion.com */

#if defined(CONFIG_MACH_OMAP3_TDM3730)
MACHINE_START(OMAP3_TDM3730, "OMAP3 TDM3730")
	.boot_params	= 0x80000100,
	.map_io	= omap3_map_io,
	.init_irq	= taotdm_init_irq,
	.init_machine	= tdm3730_init,
        .reserve	= omap_reserve,
	.timer		= &omap_timer,
MACHINE_END
#endif
