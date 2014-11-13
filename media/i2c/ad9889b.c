
/*
 * References (c = chapter, p = page):
 * REF_01 - Analog Devices, Programming Guide, AD9889B/AD9389B,
 * HDMI Transitter, Rev. A, October 2010
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/ad9889b.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

MODULE_DESCRIPTION("Analog Devices AD9389B/AD9889B video encoder driver");
MODULE_AUTHOR("Hans Verkuil <hans.verkuil@cisco.com>");
MODULE_AUTHOR("Martin Bugge <marbugge@cisco.com>");
MODULE_LICENSE("GPL");

#define MASK_AD9389B_EDID_RDY_INT   0x04
#define MASK_AD9389B_MSEN_INT       0x40
#define MASK_AD9389B_HPD_INT        0x80

#define MASK_AD9389B_HPD_DETECT     0x40
#define MASK_AD9389B_MSEN_DETECT    0x20
#define MASK_AD9389B_EDID_RDY       0x10

#define EDID_MAX_RETRIES (8)
#define EDID_DELAY 250
#define EDID_MAX_SEGM 8

/*
**********************************************************************
*
*  Arrays with configuration parameters for the AD9389B
*
**********************************************************************
*/

static i2c_client* client;
/* ------------------------ I2C ----------------------------------------------- */

static int ad9389b_rd(u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int ad9389b_wr(u8 reg, u8 val)
{
	int ret;
	int i;

	for (i = 0; i < 3; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret == 0)
			return 0;
	}
	printk("I2C Write Problem\n");
	return ret;
}

/* To set specific bits in the register, a clear-mask is given (to be AND-ed),
   and then the value-mask (to be OR-ed). */
static inline void ad9389b_wr_and_or(u8 reg, u8 clr_mask, u8 val_mask)
{
	ad9389b_wr(reg, (ad9389b_rd(reg) & clr_mask) | val_mask);
}

static inline bool ad9389b_have_hotplug()
{
	return ad9389b_rd(0x42) & MASK_AD9389B_HPD_DETECT;
}

static inline bool ad9389b_have_rx_sense()
{
	return ad9389b_rd(0x42) & MASK_AD9389B_MSEN_DETECT;
}

static void ad9389b_csc_conversion_mode(u8 mode)
{
	ad9389b_wr_and_or(0x17, 0xe7, (mode & 0x3)<<3);
	ad9389b_wr_and_or(0x18, 0x9f, (mode & 0x3)<<5);
}

static void ad9389b_csc_coeff(u16 A1, u16 A2, u16 A3, u16 A4,
			      u16 B1, u16 B2, u16 B3, u16 B4,
			      u16 C1, u16 C2, u16 C3, u16 C4)
{
	/* A */
	ad9389b_wr_and_or(0x18, 0xe0, A1>>8);
	ad9389b_wr(0x19, A1);
	ad9389b_wr_and_or(0x1A, 0xe0, A2>>8);
	ad9389b_wr(0x1B, A2);
	ad9389b_wr_and_or(0x1c, 0xe0, A3>>8);
	ad9389b_wr(0x1d, A3);
	ad9389b_wr_and_or(0x1e, 0xe0, A4>>8);
	ad9389b_wr(0x1f, A4);

	/* B */
	ad9389b_wr_and_or(0x20, 0xe0, B1>>8);
	ad9389b_wr(0x21, B1);
	ad9389b_wr_and_or(0x22, 0xe0, B2>>8);
	ad9389b_wr(0x23, B2);
	ad9389b_wr_and_or(0x24, 0xe0, B3>>8);
	ad9389b_wr(0x25, B3);
	ad9389b_wr_and_or(0x26, 0xe0, B4>>8);
	ad9389b_wr(0x27, B4);

	/* C */
	ad9389b_wr_and_or(0x28, 0xe0, C1>>8);
	ad9389b_wr(0x29, C1);
	ad9389b_wr_and_or(0x2A, 0xe0, C2>>8);
	ad9389b_wr(0x2B, C2);
	ad9389b_wr_and_or(0x2C, 0xe0, C3>>8);
	ad9389b_wr(0x2D, C3);
	ad9389b_wr_and_or(0x2E, 0xe0, C4>>8);
	ad9389b_wr(0x2F, C4);
}

static void ad9389b_csc_rgb_full2limit(bool enable)
{
	if (enable) {
		u8 csc_mode = 0;

		ad9389b_csc_conversion_mode(csc_mode);
		ad9389b_csc_coeff(4096-564, 0, 0, 256,
				  0, 4096-564, 0, 256,
				  0, 0, 4096-564, 256);
		/* enable CSC */
		ad9389b_wr_and_or(0x3b, 0xfe, 0x1);
		/* AVI infoframe: Limited range RGB (16-235) */
		ad9389b_wr_and_or(0xcd, 0xf9, 0x02);
	} else {
		/* disable CSC */
		ad9389b_wr_and_or(0x3b, 0xfe, 0x0);
		/* AVI infoframe: Full range RGB (0-255) */
		ad9389b_wr_and_or(0xcd, 0xf9, 0x04);
	}
}

static void ad9389b_set_manual_pll_gear(u32 pixelclock)
{
	u8 gear;

	/* Workaround for TMDS PLL problem
	 * The TMDS PLL in AD9389b change gear when the chip is heated above a
	 * certain temperature. The output is disabled when the PLL change gear
	 * so the monitor has to lock on the signal again. A workaround for
	 * this is to use the manual PLL gears. This is a solution from Analog
	 * Devices that is not documented in the datasheets.
	 * 0x98 [7] = enable manual gearing. 0x98 [6:4] = gear
	 *
	 * The pixel frequency ranges are based on readout of the gear the
	 * automatic gearing selects for different pixel clocks
	 * (read from 0x9e [3:1]).
	 */

	if (pixelclock > 140000000)
		gear = 0xc0; /* 4th gear */
	else if (pixelclock > 117000000)
		gear = 0xb0; /* 3rd gear */
	else if (pixelclock > 87000000)
		gear = 0xa0; /* 2nd gear */
	else if (pixelclock > 60000000)
		gear = 0x90; /* 1st gear */
	else
		gear = 0x80; /* 0th gear */

	ad9389b_wr_and_or(0x98, 0x0f, gear);
}

/* Power up/down ad9389b */
static int ad9389b_s_power(int on)
{
	const int retries = 20;
	int i;

	if (!on) {
		/* Power down */
		ad9389b_wr_and_or(0x41, 0xbf, 0x40);
		return true;
	}

	/* Power up */
	/* The ad9389b does not always come up immediately.
	   Retry multiple times. */
	for (i = 0; i < retries; i++) {
		ad9389b_wr_and_or(0x41, 0xbf, 0x0);
		if ((ad9389b_rd(0x41) & 0x40) == 0)
			break;
		ad9389b_wr_and_or(0x41, 0xbf, 0x40);
		msleep(10);
	}
	if (i == retries) {
		printk("failed to powerup the ad9389b\n");
		ad9389b_s_power(0);
		return false;
	}
	if (i > 1)
		printk("needed %d retries to powerup the ad9389b\n", i);

	/* Select chip: AD9389B */
	ad9389b_wr_and_or(0xba, 0xef, 0x10);

	/* Reserved registers that must be set according to REF_01 p. 11*/
	ad9389b_wr_and_or(0x98, 0xf0, 0x07);
	ad9389b_wr(0x9c, 0x38);
	ad9389b_wr_and_or(0x9d, 0xfc, 0x01);

		//ad9389b_wr(sd, 0xa2, pdata->diff_data_drive_strength);
		ad9389b_wr(0xa2, 0x87);

		//ad9389b_wr(sd, 0xa3, pdata->diff_clk_drive_strength);
		ad9389b_wr(0xa3, 0x87);

	ad9389b_wr(0x0a, 0x01);
	ad9389b_wr(0xbb, 0xff);

	/* Set number of attempts to read the EDID */
	ad9389b_wr(0xc9, 0xf);
	return true;
}

/* ------------------------------ VIDEO OPS ------------------------------ */

/* Initial setup of AD9389b */

/* Configure hdmi transmitter. */
static void ad9389b_setup()
{
	/* Input format: RGB 4:4:4 */
	ad9389b_wr_and_or(0x15, 0xf1, 0x0);
	/* Output format: RGB 4:4:4 */
	ad9389b_wr_and_or(0x16, 0x3f, 0x0);
	/* CSC fixed point: +/-2, 1st order interpolation 4:2:2 -> 4:4:4 up
	   conversion, Aspect ratio: 16:9 */
	ad9389b_wr_and_or(0x17, 0xe1, 0x0e);
	/* Disable pixel repetition and CSC */
	ad9389b_wr_and_or(0x3b, 0x9e, 0x0);
	/* Output format: RGB 4:4:4, Active Format Information is valid. */
	ad9389b_wr_and_or(0x45, 0xc7, 0x08);
	/* Underscanned */
	ad9389b_wr_and_or(0x46, 0x3f, 0x80);
	/* Setup video format */
	ad9389b_wr(0x3c, 0x0);
	/* Active format aspect ratio: same as picure. */
	ad9389b_wr(0x47, 0x80);
	/* No encryption */
	ad9389b_wr_and_or(0xaf, 0xef, 0x0);
	/* Positive clk edge capture for input video clock */
	ad9389b_wr_and_or(0xba, 0x1f, 0x60);
}

static int ad9389b_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	const struct v4l2_dv_timings dv1080p60 = V4L2_DV_BT_CEA_1920X1080P60;
	int err = -EIO;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk("i2c check functionality failed\n");
		return -EIO;
	}
	/* Platform data */
	if (pdata == NULL) {
		printk("No platform data!\n");
		err = -ENODEV;
		goto err_free;
	}

	ad9389b_init_setup(sd);

	return 0;

err_unreg:
	i2c_unregister_device(state->edid_i2c_client);
err_free:
	return err;
}

/* ----------------------------------------------------------------------- */

static int ad9389b_remove(struct i2c_client *client)
{
	ad9389b_init_setup();
	i2c_unregister_device(state->edid_i2c_client);
	return 0;
}

/* ----------------------------------------------------------------------- */

static struct i2c_device_id ad9389b_id[] = {
	{ "ad9389b", V4L2_IDENT_AD9389B },
	{ "ad9889b", V4L2_IDENT_AD9389B },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad9389b_id);

static struct i2c_driver ad9389b_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ad9889b",
	},
	.probe = ad9389b_probe,
	.remove = ad9389b_remove,
	.id_table = ad9389b_id,
};

module_i2c_driver(ad9389b_driver);
