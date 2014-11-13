
/* Support for TechNexion LCD panels 
   This file is licensed under GPL.
   linuxfae@technexion.com
*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <plat/display.h>

//+uki
#define p347_GPIO_BACKLIGHT_POWER	138
//-uki

enum {
        TNLCD_PIXELCLOCK,	/* Pixel clock, in Hz */

        TNLCD_HRES,		/* Horizontal resolution, pixels */
        TNLCD_HFP,		/* Horizontal front porch */
        TNLCD_HBP,		/* Horizontal back porch */
        TNLCD_HSW,		/* Horizontal what? */

        TNLCD_VRES,	
        TNLCD_VFP,
        TNLCD_VBP,
        TNLCD_VSW,

        TNLCD_DATALINES,	/* Number of data lines (typically 24, but 18 exists too) */
        TNLCD_BPP,		/* Bits per pixel to use (unused parameter for now) */
        
        TNLCD_NOF_DATAFIELDS	/* Must be last in enum! */
};

/* The initial value of the data array is the tmings etc for Innolux AT070TN93
   However, these get overwritten when the module paramter tnlcd.data is given. */
static int tnlcd_data[TNLCD_NOF_DATAFIELDS] =
        { 33260, 800, 210, 46, 1, 480, 22, 23, 1,  24, 32 };	/* Overwritten */
/* Read parameter array */
module_param_array(tnlcd_data, int, NULL, S_IRUGO);

struct tnlcd_panels {
	const char *name;
	struct omap_video_timings panel_timings;
};

static const struct tnlcd_panels tnlcd_paneldb[] = 
{
	/* name_string,	x_res,y_res,pixclk, hsw, hfp, hbp, vsw, vfp, vbp*/
	{"litemax_dlf1095_enn_a01",	{1024, 768, 65000, 5, 50, 265, 2, 16, 20}},
	{"hannstar_hsd101pfw1_a00",	{1024, 600, 45000, 5, 43, 104, 5, 20, 24}},
	{"auo_b089aw01",		{1024, 600, 54000, 42, 48, 320, 5, 5, 20}},
	{"lg_lb043wq2",			{480, 272, 9000, 42, 3, 2, 11, 3, 2}},
	{"innolux_at070tn94",		{800, 480, 33300, 1, 210, 46, 1, 22, 23}},
	{"auo-g065vn01",		{640,480, 25200, 1,80,80,1,25,20}},
	{"promate-97g121s1n4f",		{800,600,39800,1,128,128,1,14,14}},
	{"auo-g150xg01",		{1024,768,65000,1,160,160,1,19,19}},
	{"auo-g070vw01",		{800,480,33260,1,128,128,1,20,20}},
	{"promate-97g084s5n5f",		{800,600,39800,1,128,128,1,14,14}},
	{"promate-97g104s2n2f",		{800,600, 40000, 1,128,128,1,14,14}},
};

static int tnlcd_power_on(struct omap_dss_device *dssdev)
{
	int r;

	//uki
	printk("OOOOOOOOOOOOOOOOOOOO tnlcd_power_on\n");
	gpio_set_value(p347_GPIO_BACKLIGHT_POWER,1);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void tnlcd_power_off(struct omap_dss_device *dssdev)
{
	//uki
	printk("OOOOOOOOOOOOOOOOOOOO tnlcd_power_off\n");
	gpio_set_value(p347_GPIO_BACKLIGHT_POWER,0);
	
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);
}

static int tnlcd_probe(struct omap_dss_device *dssdev)
{
	int data_line = tnlcd_data[TNLCD_DATALINES];

	dssdev->panel.config |= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;

	dssdev->panel.timings.pixel_clock = tnlcd_data[TNLCD_PIXELCLOCK];
	dssdev->panel.timings.x_res       = tnlcd_data[TNLCD_HRES];
	dssdev->panel.timings.hfp         = tnlcd_data[TNLCD_HFP];
	dssdev->panel.timings.hbp         = tnlcd_data[TNLCD_HBP];
	dssdev->panel.timings.hsw         = tnlcd_data[TNLCD_HSW];
	dssdev->panel.timings.y_res       = tnlcd_data[TNLCD_VRES];
	dssdev->panel.timings.vfp         = tnlcd_data[TNLCD_VFP];
	dssdev->panel.timings.vbp         = tnlcd_data[TNLCD_VBP];
	dssdev->panel.timings.vsw         = tnlcd_data[TNLCD_VSW];

	if((data_line != 12)&&(data_line != 16)&&(data_line != 18)&&(data_line != 24))
		tnlcd_data[TNLCD_DATALINES] = 24;

	dssdev->phy.dpi.data_lines        = tnlcd_data[TNLCD_DATALINES];
	dssdev->ctrl.pixel_size           = tnlcd_data[TNLCD_BPP];

	return 0;
}

static void tnlcd_remove(struct omap_dss_device *dssdev)
{
}

static int tnlcd_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = tnlcd_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void tnlcd_disable(struct omap_dss_device *dssdev)
{
	tnlcd_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int tnlcd_suspend(struct omap_dss_device *dssdev)
{
	tnlcd_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int tnlcd_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = tnlcd_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void tnlcd_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void tnlcd_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int tnlcd_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver tnlcd_driver = {
	.probe		= tnlcd_probe,
	.remove		= tnlcd_remove,

	.enable		= tnlcd_enable,
	.disable	= tnlcd_disable,
	.suspend	= tnlcd_suspend,
	.resume		= tnlcd_resume,

	.set_timings	= tnlcd_set_timings,
	.get_timings	= tnlcd_get_timings,
	.check_timings	= tnlcd_check_timings,

	.driver         = {
		.name	= "tnlcd",
		.owner  = THIS_MODULE,
	},
};

static int __init tnlcd_drv_init(void)
{
	gpio_request(p347_GPIO_BACKLIGHT_POWER,"gpio_backlight_power\n");
	gpio_direction_output(p347_GPIO_BACKLIGHT_POWER,1);

	return omap_dss_register_driver(&tnlcd_driver);
}

static void __exit tnlcd_drv_exit(void)
{
	gpio_free(p347_GPIO_BACKLIGHT_POWER);
	omap_dss_unregister_driver(&tnlcd_driver);
}

module_init(tnlcd_drv_init);
module_exit(tnlcd_drv_exit);
MODULE_LICENSE("GPL");
