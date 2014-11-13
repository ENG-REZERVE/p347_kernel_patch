/**
 * twl4030-pwrbutton.c - TWL4030 Power Button Input Driver
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 * Several fixes by Felipe Balbi <felipe.balbi@nokia.com>
 * Adapted for p347 board by Konstantin Utkin <kostaui@mail.ru>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/atomic.h>
//+uki
//#include <linux/reboot.h>
//-uki

#define STATE_BUTTON_PRESSED		0x1
#define STATE_BUTTON_RELEASED		0x0

#define PWR_PWRON_IRQ (1 << 0)

#define STS_HW_CONDITIONS 0xf

#define CHECK_VALUE_E(_mod_,_reg_)\
    reg_val = 0;\
    ret = twl_i2c_read_u8(_mod_,&reg_val,_reg_);\
    printk("ret read=%d, verify read=0x%02x\n",ret,reg_val)

static atomic_t		filter_msec;
static atomic_t		time_out;
static atomic_t		prev_value;
static struct delayed_work but_check_work;
static atomic_long_t	work_ptr;
static struct kobject*	sysfs_object = NULL;

static ssize_t show_fm(struct device* dev, struct device_attribute* attr, char* buf)
{
    return sprintf(buf, "%d\n", atomic_read(&filter_msec));
}

static ssize_t store_fm(struct device* dev, struct device_attribute* attr,
    const char* buf, size_t count) 
{
    long val = simple_strtol(buf, NULL, 10);
    atomic_set(&filter_msec,val);
    return count;
}

//sysfs access to filter_msec value
static struct device_attribute dev_attr_pb_pbfiltmsec = {
    .attr = {
	.name = "pbfiltmsec",
//	.owner = THIS_MODULE,
	.mode = S_IWUSR | S_IRUGO,
    },
    .show = show_fm,
    .store = store_fm,
};

//------------------------------------------------------------------------

//void delayed_function( unsigned long data ) {
void delayed_function( struct work_struct* work ) {
    int err;
    u8 value;
    u8 v2;
    struct input_dev *pwr = (struct input_dev*)atomic_long_read(&work_ptr);
    atomic_set(&time_out,1);
    
    err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &value,
				STS_HW_CONDITIONS);
    if (!err) {
	v2 = value & PWR_PWRON_IRQ;
	//printk("delayed check, value = 0x%x\n",v2);
	if (v2 == STATE_BUTTON_PRESSED) {//report key press if it still pressed
	    //printk("report power key press after filtering delay\n");
	    input_report_key(pwr, KEY_POWER, STATE_BUTTON_PRESSED);
	    input_sync(pwr);
	}
    }
}

static irqreturn_t powerbutton_irq(int irq, void *_pwr)
{
	struct input_dev *pwr = _pwr;
	int err;
	u8 value;
	u8 v2;
	u8 reg_val;
	int ret;

	//if (atomic_read(&int_ignore) != 0) return IRQ_HANDLED;

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &value,
				STS_HW_CONDITIONS);
	if (!err)  {
		v2 = value & PWR_PWRON_IRQ;
		//printk("_POWERBUTTON IRQ, value = 0x%x\n",v2);
		if (atomic_read(&filter_msec) != 0) {
		    if (v2 == STATE_BUTTON_PRESSED) {
			if (atomic_read(&prev_value) == STATE_BUTTON_PRESSED) { //last value was "pressed" too
			    //then cancel current scheduled work and plan it again
			    cancel_delayed_work_sync(&but_check_work);
			} //else just plan
			atomic_long_set(&work_ptr,(unsigned long)pwr);
			schedule_delayed_work(&but_check_work,msecs_to_jiffies(atomic_read(&filter_msec)));
			atomic_set(&time_out,0);
		    } else {		//button released
			if (atomic_read(&prev_value) == STATE_BUTTON_PRESSED) {
			    //check time
			    if (atomic_read(&time_out) == 1) {
				//printk("report released!\n");
				input_report_key(pwr, KEY_POWER, v2);
				input_sync(pwr);
			    } else {
				cancel_delayed_work_sync(&but_check_work);
				atomic_set(&time_out,1);
			    }
			} //else do nothing
		    }
		    //remember value
		    atomic_set(&prev_value,v2);
		} else {
		    //printk("report direct\n");
		    input_report_key(pwr, KEY_POWER, v2);
		    input_sync(pwr);
		}
 /*
    printk("HW_STATUS_REGISTER:  ");
    CHECK_VALUE_E(TWL4030_MODULE_PM_MASTER,0x45);
    
    printk(" SYSEN DEV_GRP, TYPE and REMAP registers: \n");
    CHECK_VALUE_E(TWL4030_MODULE_PM_RECEIVER,0xE3);
    CHECK_VALUE_E(TWL4030_MODULE_PM_RECEIVER,0xE3);
    CHECK_VALUE_E(TWL4030_MODULE_PM_RECEIVER,0xE3);
    
    twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER,0x01,0x46);
    twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER,0x01,0x47);
    twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER,0x01,0x48);

    reg_val = 0x0D; //MADC_IMR1 register value
    twl_i2c_write_u8(TWL4030_MODULE_MADC,reg_val,TWL4030_MADC_IMR1);
    CHECK_VALUE_E(TWL4030_MODULE_MADC,TWL4030_MADC_IMR1);
 */
	} else {
		dev_err(pwr->dev.parent, "twl4030: i2c error %d while reading"
			" TWL4030 PM_MASTER STS_HW_CONDITIONS register\n", err);
	}

	return IRQ_HANDLED;
}

static int __init twl4030_pwrbutton_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int irq = platform_get_irq(pdev, 0);
	int err;

	atomic_set(&filter_msec,200);
	atomic_set(&prev_value,STATE_BUTTON_RELEASED);
	atomic_set(&time_out,1);

	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	pwr->evbit[0] = BIT_MASK(EV_KEY);
	pwr->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	pwr->name = "twl4030_pwrbutton";
	pwr->phys = "twl4030_pwrbutton/input0";
	pwr->dev.parent = &pdev->dev;

	err = request_threaded_irq(irq, NULL, powerbutton_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl4030_pwrbutton", pwr);
	if (err < 0) {
		dev_dbg(&pdev->dev, "Can't get IRQ for pwrbutton: %d\n", err);
		goto free_input_dev;
	}

	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto free_irq;
	}

	platform_set_drvdata(pdev, pwr);
	INIT_DELAYED_WORK(&but_check_work,delayed_function);
	
	//sysfs
	//sysfs_object = kobject_create_and_add("p347_power_key",&pdev->dev.kobj);
	sysfs_object = kobject_create_and_add("p347_power_key",NULL);
	if (!sysfs_object) {
	    printk("Can't create sysfs directory for p347 power key\n");
	} else {
	    err = sysfs_create_file(sysfs_object,&dev_attr_pb_pbfiltmsec.attr);
    	    printk("power button probe sysfs_create_file ret %d\n",err);
        }
        
	return 0;

free_irq:
	free_irq(irq, NULL);
free_input_dev:
	input_free_device(pwr);
	return err;
}

static int __exit twl4030_pwrbutton_remove(struct platform_device *pdev)
{
	struct input_dev *pwr = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	free_irq(irq, pwr);
	input_unregister_device(pwr);
	cancel_delayed_work_sync(&but_check_work);
	sysfs_remove_file(sysfs_object,&dev_attr_pb_pbfiltmsec.attr);
	if (sysfs_object != NULL) {
	    kobject_put(sysfs_object);
	    sysfs_object = NULL;
	}

	return 0;
}

static struct platform_driver twl4030_pwrbutton_driver = {
	.remove		= __exit_p(twl4030_pwrbutton_remove),
	.driver		= {
		.name	= "twl4030_pwrbutton",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_pwrbutton_init(void)
{
	return platform_driver_probe(&twl4030_pwrbutton_driver,
			twl4030_pwrbutton_probe);
}
module_init(twl4030_pwrbutton_init);

static void __exit twl4030_pwrbutton_exit(void)
{
	platform_driver_unregister(&twl4030_pwrbutton_driver);
}
module_exit(twl4030_pwrbutton_exit);

MODULE_ALIAS("platform:twl4030_pwrbutton");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver <peter.de-schrijver@nokia.com>");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");
