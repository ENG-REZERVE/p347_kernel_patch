/*
  p347 hardware monitoring driver
  
  1) TPS65930 ADC inputs monitoring: incorrect operation
  2) Internal SoC temperature sensor monitoring: OK 
    (requires volts-to-value conversion table with floating point)
  3) Interface for on-board beeper: OK
  4) stc3105iqt battery charge monitoring: OK

*/

#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/i2c.h>

#include <linux/i2c/twl.h>

#include <linux/reboot.h>

#include "p347_pwr.h"

#define PROGRAM_AUTHOR			"Konstantin Utkin"
#define p347_DRV_VERSION		"p347 HWMON driver " __DATE__ " " __TIME__ " by " PROGRAM_AUTHOR

#define CONV_WAIT_MSEC			1000
#define CONV_MAX_WAIT_CNT		3
#define CONV_INTERVAL_MSEC		1000

#define MONITORING_INTERVAL_MSEC	1000

#define _P347_DEBUG_

#ifdef _P347_DEBUG_
#define _PDBA(_x_, args...)	printk("_PDB: "_x_, args)
#define _PDB(_x_)		printk("_PDB: "_x_)
#else
#define _PDBA(_x_, args...)	do {}
#define _PDB(_x_)		do {}
#endif

#define TURN_CHARGER_ON		gpio_set_value(p347_GPIO_CHARGER_SHDN,1)
#define TURN_CHARGER_OFF	gpio_set_value(p347_GPIO_CHARGER_SHDN,0)

#define BAT_PERCENT_CRITICAL	2
#define BAT_PERCENT_ALARM	10
#define BAT_PERCENT_HALF	50

typedef struct {
    //private
    struct i2c_client 	*client;
    int 		irq; //for alarm, todo
    spinlock_t		lock;
    //public
    t_batmon_data	bat;
    t_batmon_params	bpar;
}t_bcharge_data;

static t_bcharge_data	stc_data;
//static int bat_first_read = 1;
atomic_t bat_first_read;
static int real_charge_uah = 0;
static signed int prev_charge_uah = 0;

//static struct timer_list conv_timer;
static struct delayed_work conv_work;
unsigned long scm_base_addr;
static struct clk *dpll3_clk = NULL;

//==========================================================================
//==========================================================================		I2C functions
//==========================================================================

int stc_read_single_reg(unsigned char dev_addr, unsigned char reg_addr, void* val)
{
    struct i2c_msg xfer[2];
    int ret;
    u8 buf[2];
    buf[0] = reg_addr;
    
	xfer[0].addr = dev_addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &buf[0];
    
	xfer[1].addr = dev_addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = 1;
	xfer[1].buf = val;
    
	ret = i2c_transfer(stc_data.client->adapter, xfer, ARRAY_SIZE(xfer));
	//printk("addr %d ret %d val %d\n",i,ret,(unsigned char *)(xfer[1].buf)[0]);
    
    return ret;
}

int stc_write_base(int new_base) {
    int ret;
    unsigned short val = uAh_to_bits(new_base);

	u8 buf[3];
	buf[0] = 10;
	buf[1] = val & 0xFF;//low
	buf[2] = (val >> 8) & 0xFF;//high

	ret = i2c_master_send(stc_data.client,buf,3);

    return ret;
}

int stc_read_reg_pack(void) {
    struct i2c_msg xfer[2];
    int ret;
    unsigned long flags;
    unsigned short hex_charge,hex_voltage,hex_current,hex_base;
    int tmpi;
    signed int delta_charge;
    unsigned long tmpl;
    
    u8 input[17];
    u8 output = 0;
    
    xfer[0].addr = stc_data.client->addr;
    xfer[0].flags = 0;
    xfer[0].len = 1;
    xfer[0].buf = &output;
    
    xfer[1].addr = stc_data.client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = 17;
    xfer[1].buf = &input[0];
    
    ret = i2c_transfer(stc_data.client->adapter, xfer, ARRAY_SIZE(xfer));
    
    if (ret>0) {
	hex_charge = (input[3] << 8)+input[2];
	hex_voltage = (input[9] << 8)+input[8];
	hex_current = (input[7] << 8)+input[6];
	hex_base = (input[11] << 8)+input[10];
    
	spin_lock_irqsave(&stc_data.lock,flags);
	//monitor values
	stc_data.bat.hex_current_charge = hex_charge;
	stc_data.bat.val_current_charge_uAh = uVh_bits_to_uAh(hex_charge);
	stc_data.bat.val_base_charge_uAh = uVh_bits_to_uAh(hex_base);
	stc_data.bat.val_icharge_uA = bits_to_uA(hex_current);
	//fix zero current!!!
	if (stc_data.bat.val_icharge_uA == 0)
	    stc_data.bat.val_icharge_uA = 1;

	tmpl = stc_data.bpar.voltage_multiplier*bits_to_mV(hex_voltage); 
	stc_data.bat.val_ucharge_mV = tmpl / 1000;
	stc_data.bat.is_bm_connected = 1;
	stc_data.bat.no_connect_counter = 0;
	//pins
	//printk("fullchg = %d, fastchg =%d\n",(1 - gpio_get_value(p347_GPIO_CHARGER_FULLCHG)),(1 - gpio_get_value(p347_GPIO_CHARGER_FASTCHG)));
	//printk("icharge = %d, mincur = %d\n",stc_data.bat.val_icharge_uA,stc_data.bpar.minimum_charge_current_uA);
	stc_data.bat.shdn_state = gpio_get_value(p347_GPIO_CHARGER_SHDN);
	if ((stc_data.bat.val_icharge_uA > stc_data.bpar.minimum_charge_current_uA) && (stc_data.bat.shdn_state != 0)) {
	    stc_data.bat.is_charging = 1;
	} else {
	    stc_data.bat.is_charging = 0;
	}
	stc_data.bat.is_fault = 1 - gpio_get_value(p347_GPIO_CHARGER_FAULT);
	
    	//detect outer power supply
    	if (stc_data.bat.val_icharge_uA > 0) {
	    stc_data.bat.outer_power_presense = 1;
    	} else {
    	    stc_data.bat.outer_power_presense = 0;
    	}
    	
    	//detect battery
    	if (stc_data.bat.val_ucharge_mV < stc_data.bpar.minimum_possible_battery_voltage_mV) {
    	    //stc_data.bat.charge_percent = 0;
            //stc_data.bat.minutes_left = 0xFFFF;
	    stc_data.bat.battery_presense = 0;
    	} else {
    	    //stc_data.bat.charge_percent = 100*stc_data.bat.val_current_charge_uAh/stc_data.bpar.max_battery_charge_uAh;
            //stc_data.bat.minutes_left = 0xFFFF;
	    stc_data.bat.battery_presense = 1;
    	}
    	
    	spin_unlock_irqrestore(&stc_data.lock,flags);


	if (atomic_read(&bat_first_read) == 1) { //----------------------------------------------------------------FIRST READ!
	
	    if (stc_data.bat.val_ucharge_mV <= stc_data.bpar.alarm_battery_voltage_mV) {//between alarm and critical
    		tmpi = (stc_data.bat.val_ucharge_mV - stc_data.bpar.critical_battery_voltage_mV)*(BAT_PERCENT_ALARM-BAT_PERCENT_CRITICAL);
    		//printk("ac tmpi1=%d\n",tmpi);
    		tmpi /= (stc_data.bpar.alarm_battery_voltage_mV - stc_data.bpar.critical_battery_voltage_mV);
    		//printk("ac tmpi2=%d\n",tmpi);
    		stc_data.bat.charge_percent = BAT_PERCENT_CRITICAL + tmpi;
    	    } else {
    	        if (stc_data.bat.val_ucharge_mV <= stc_data.bpar.half_battery_voltage_mV) { //between half and alarm
    		    tmpi = (stc_data.bat.val_ucharge_mV - stc_data.bpar.alarm_battery_voltage_mV)*(BAT_PERCENT_HALF-BAT_PERCENT_ALARM);
    		    //printk("ha tmpi1=%d\n",tmpi);
    		    tmpi /= (stc_data.bpar.half_battery_voltage_mV - stc_data.bpar.alarm_battery_voltage_mV);
    		    //printk("ha tmpi2=%d\n",tmpi);
    		    stc_data.bat.charge_percent = BAT_PERCENT_ALARM + tmpi;
    		} else { //between full and half
    		    tmpi = (stc_data.bat.val_ucharge_mV - stc_data.bpar.half_battery_voltage_mV)*(100-BAT_PERCENT_HALF);
    		    //printk("fh tmpi1=%d\n",tmpi);
    		    tmpi /= (stc_data.bpar.full_battery_voltage_mV - stc_data.bpar.half_battery_voltage_mV);
    		    //printk("fh tmpi2=%d\n",tmpi);
    		    stc_data.bat.charge_percent = BAT_PERCENT_HALF + tmpi;
    		}
    	    }
	
	    tmpi = stc_data.bpar.max_battery_charge_uAh * stc_data.bat.charge_percent;
	    real_charge_uah = tmpi / 100;
	    //stc_write_base(0);
	    prev_charge_uah = stc_data.bat.val_current_charge_uAh;
	    //prev_charge_uah = 0;
	    printk("FIRST READ: estimated charge %d percent, write %d to real charge uah\n",stc_data.bat.charge_percent,real_charge_uah);
	    
	    atomic_set(&bat_first_read,0);
	} else {                   //----------------------------------------------------------------COMMON READ
	    delta_charge = stc_data.bat.val_current_charge_uAh - prev_charge_uah;
	    //printk("prev=%d, current=%d\n",prev_charge_uah,stc_data.bat.val_current_charge_uAh);
	    prev_charge_uah = stc_data.bat.val_current_charge_uAh;
	    real_charge_uah += delta_charge;
	    //printk("real = %d\n",real_charge_uah);

	    //correcting
	    if (real_charge_uah > stc_data.bpar.max_battery_charge_uAh)
		real_charge_uah = stc_data.bpar.max_battery_charge_uAh;

	    tmpi = 100*real_charge_uah;
	    stc_data.bat.charge_percent = tmpi / stc_data.bpar.max_battery_charge_uAh;
	    //printk("REAL CHARGE %d, %d percent\n",real_charge_uah,stc_data.bat.charge_percent);
	}

	//-------------------------------------------------------------------------------------------CHARGER MANAGE
    	if (stc_data.bat.battery_presense != 0) {
    	    //force system shutdown
    	    if ((stc_data.bat.val_ucharge_mV < stc_data.bpar.critical_battery_voltage_mV) && (stc_data.bat.outer_power_presense == 0)) {
    		printk("ALARM! Extremely low battery voltage (%d mV), shutting down system\n",stc_data.bat.val_ucharge_mV);
    		//reboot(RB_POWER_OFF);
    		//machine_power_off();
    		kernel_power_off();
    	    }
    	    //force charger off
    	    if (stc_data.bat.is_charging != 0) {
    		if ((stc_data.bat.val_icharge_uA <= stc_data.bpar.minimum_charge_current_uA) ||
    		    (stc_data.bat.val_ucharge_mV >= stc_data.bpar.full_battery_voltage_mV)) {
    		    printk("Battery is fully charged, force charger off\n");
    		    TURN_CHARGER_OFF;
    		}
    	    }
    	    if ((stc_data.bat.outer_power_presense == 0) && (stc_data.bat.shdn_state == 1)) {//shdn=1 while outer power is off
    		printk("No outer power, force charger off\n");
    		TURN_CHARGER_OFF;
    	    }
    	    
    	    //force charger on
    	    if ((stc_data.bat.is_charging == 0) &&
    	        (stc_data.bat.outer_power_presense != 0) &&
    	        (stc_data.bat.charge_percent < stc_data.bpar.b_mincharge_percent)) {
    	        
    	        printk("force charger on\n");
    	        TURN_CHARGER_OFF;
    		msleep(1);
    		TURN_CHARGER_ON;
    	    }
    	    
    	    //turn on-off alarm on very low battery voltage
    	    if (stc_data.bat.charge_percent < stc_data.bpar.b_alarm_percent) {//compare with some hysteresis
    		stc_data.bat.alarm_low_voltage = 1;
    	    } else if (stc_data.bat.charge_percent > stc_data.bpar.b_alarm_percent) {
    		stc_data.bat.alarm_low_voltage = 0;
    	    }

    	    //minutes forecast
    	    if (!stc_data.bat.outer_power_presense) {
    		tmpl = stc_data.bpar.max_battery_charge_uAh / 100;
    		tmpl *= 60;
    		tmpl *= stc_data.bat.charge_percent;

		if (stc_data.bat.val_icharge_uA > 0) {
    		    tmpl /= stc_data.bat.val_icharge_uA;
    		} else {
    		    tmpl /= (-stc_data.bat.val_icharge_uA);
		}
    
    		stc_data.bat.minutes_left = tmpl;

    		if (stc_data.bat.minutes_left > 1000) { //is over possible limit
    		    stc_data.bat.minutes_left = 1000;
    		}
    	    } else { //have outer power
    		stc_data.bat.minutes_left = 9999;
    	    }
    	} else { //no battery
    	    if (stc_data.bat.shdn_state == 1) {
    		TURN_CHARGER_OFF;
    	    }
    	    stc_data.bat.minutes_left = 9999;
    	}
    	
    } else {
	spin_lock_irqsave(&stc_data.lock,flags);
	if (stc_data.bat.no_connect_counter >= MAX_NO_CONNECT_COUNTER) {
	    stc_data.bat.is_bm_connected = 0;
	} else {
	    stc_data.bat.no_connect_counter++;
	}
	spin_unlock_irqrestore(&stc_data.lock,flags);
    
	printk("Battery monitor is not connected!\n");
    }

    return ret;
}

int stc_run(void) {
    int ret;
    u8 buf[2];
    buf[0] = 0; buf[1] = 0x10; //mode, GG_RUN
    
    ret = i2c_master_send(stc_data.client,buf,2);

    return ret;
}


//==========================================================================
//==========================================================================		BEEPS
//==========================================================================

typedef struct {
    struct work_struct my_work;
    spinlock_t lock;
    unsigned short len_ms;
} my_ws_beep_t;

static struct workqueue_struct *beep_workqueue;
static my_ws_beep_t beep_work;

static void do_beep(struct work_struct* wbeep) {
    unsigned long flags;
    unsigned short len_ms;
    my_ws_beep_t* wb = container_of(wbeep, my_ws_beep_t, my_work);
    
    spin_lock_irqsave(&wb->lock,flags);
    len_ms = wb->len_ms;
    spin_unlock_irqrestore(&wb->lock,flags);
    
    //printk("beep for %d msec\n",len_ms);
    if (len_ms > 0) {
	gpio_set_value(p347_GPIO_BEEP,1);
	msleep(len_ms);
	gpio_set_value(p347_GPIO_BEEP,0);
    }
}

//==========================================================================
//==========================================================================		TPS65930 ADCIN0
//==========================================================================

//void do_conversion(unsigned long data) {
    //u8 reg_val;
    //unsigned char d_cnt;
    //unsigned char rreg;

//    unsigned long regl;
    //unsigned long tmpl=0;
    //int ret,i = 0;
    //unsigned short result = 0;
/*
    //_PDB("-----------------------------TEMP SENSOR\n");
	__raw_writel(0x200,scm_base_addr+0x2524);
	do {
	    regl = __raw_readl(scm_base_addr+0x2524);
	    if (regl & 0x100) {
		__raw_writel(0x000,scm_base_addr+0x2524);
		break;
	    }
	    tmpl++;
	} while (tmpl < 100000);
	if (tmpl < 100000) {
	    //_PDBA("tmpl1=%d\n",tmpl); //1200-1500 (experimentally)
	    tmpl = 0;
	    do {
		regl = __raw_readl(scm_base_addr+0x2524);
		if ((regl & 0x100) == 0) {
		    //_PDBA("tmpl2=%d\n",tmpl);
		    //_PDBA("register value = %4x\n",regl);
		    //_PDBA("register value = %4x, CPU temp ~= %5.1f\n",regl,dm3730_temp_adc_table[(regl&0xFF)]);
		    break;
		}
		tmpl++;
	    } while (tmpl < 100000);
	    if (tmpl >= 100000)
		_PDB("Waiting EOCZ bit ->0 timeout\n");
	} else {
	    _PDB("Waiting EOCZ bit ->1 timeout\n");
	}
	//_PDB("-----------------------------------------\n");
	 */
    //---------------------------------------------------------------------------


/*
#ifdef READ_TWL4030_ADC
    _PDB("-----------------------------TWL4030_ADC\n");

    reg_val = 0;
    ret = twl_i2c_read_u8(TWL4030_MODULE_MADC,&reg_val,0x12); //verify
    _PDBA("CTRL_SW1 first val = 0x%2x\n",reg_val);
    reg_val = 0x20; //CTRL_SW set SW1
    ret = twl_i2c_write_u8(TWL4030_MODULE_MADC,reg_val,0x12);
    if (ret!=0) {
	_PDBA("conversion start failed, i2c_write ret=%d\n",ret);
	schedule_delayed_work(&conv_work,msecs_to_jiffies(CONV_INTERVAL_MSEC));
	//return ret;
	return;
    }
    reg_val = 0;
    ret = twl_i2c_read_u8(TWL4030_MODULE_MADC,&reg_val,0x12); //verify
    _PDBA("CTRL_SW1 verify = 0x%2x\n",reg_val);
    
    do {
	msleep(CONV_WAIT_MSEC);
	i++;
	//look for INT1 flag
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC,&reg_val,TWL4030_MADC_ISR1);
	if (ret!=0) {
	    _PDBA("conversion process failed, i2c_read ret=%d\n",ret);
	    schedule_delayed_work(&conv_work,msecs_to_jiffies(CONV_INTERVAL_MSEC));
	    //return ret;
	    return;
	}
	
	if (reg_val & 0x2) {
	    _PDB("SW1_ISR1 detected\n"); //Flag is clear-on-read by default (MADC_SIH_CTRL register)
	    break;
	} else {
	    _PDBA("MADC_ISR1 value=0x%02x\n",reg_val);
	}
	
	//+test
	reg_val = 0;
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC,&reg_val,0x12); //verify
	_PDBA("CTRL_SW1 check = 0x%2x\n",reg_val);
	
	//-test
    } while (i<CONV_MAX_WAIT_CNT);
    
    if (i>=CONV_MAX_WAIT_CNT) {
	_PDB("Timeout while waiting conversion completion\n");
	//schedule_delayed_work(&conv_work,msecs_to_jiffies(CONV_INTERVAL_MSEC));
	//return;
	//return -ENODEV; //cannot detect sw1_isr1
    }
    //read value
    ret = twl_i2c_read(TWL4030_MODULE_MADC,&result,0x37,2); //GPCH0 MSB and LSB
    //ret = twl_i2c_read(TWL4030_MODULE_MADC,&result,0x3B,2); //GPCH2 MSB and LSB
    if (ret!=0) {
	_PDBA("read conversion result failed, i2c_read ret=%d\n",ret);
	schedule_delayed_work(&conv_work,msecs_to_jiffies(CONV_INTERVAL_MSEC));
	//return ret;
	return;
    }
    
    //_PDBA("ADCIN2 read = 0x%04x\n",result);
    _PDBA("ADCIN0 read = 0x%04x\n",result);
#endif
*/

//}

void do_monitoring_work(unsigned long data) {
    //BATTERY MONITOR
    stc_read_reg_pack();
    //INPUT DEVICE LISTENING

    schedule_delayed_work(&conv_work,msecs_to_jiffies(MONITORING_INTERVAL_MSEC));
}

int start_monitoring(void) {
    int ret = 0;
    
    atomic_set(&bat_first_read,1);
    
    ret = stc_run();
    _PDBA("stc_run ret %d\n",ret);

    stc_write_base(0);

    INIT_DELAYED_WORK(&conv_work,do_monitoring_work);
    ret = schedule_delayed_work(&conv_work,msecs_to_jiffies(MONITORING_INTERVAL_MSEC));
    _PDBA("schedule_work ret %d\n",ret);

    return ret;
}

void stop_monitoring(void) {
/*
    int ret = del_timer_sync(&conv_timer);
    _PDBA("del_timer ret %d\n",ret);
*/
    int ret = cancel_delayed_work_sync(&conv_work);
    _PDBA("cancel_work ret %d\n",ret);
}

//==========================================================================
//==========================================================================
//==========================================================================

//==========================================================================
//==========================================================================		COMMON
//==========================================================================

static int p347_pwr_open(struct inode *inode, struct file *filp)
{
    _PDBA("p347_pwr_open: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);

    return nonseekable_open(inode,filp);
}

//==========================================================================

static int p347_pwr_release(struct inode *inode, struct file *filp)
{
    _PDBA("p347_pwr_release: inode=0x%p, filp=0x%p \n",(void*)inode,(void*)filp);

    return 0;
}

//==========================================================================
//==========================================================================		I2C
//==========================================================================


static unsigned int mv_estimation[101];

//==========================================================================
//==========================================================================
//==========================================================================

static long p347_pwr_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
    unsigned long reg_val;
    t_batmon_data* user_bat_data;
    t_batmon_params* user_bat_params;
    unsigned long flags;
    
    int ret;
    int tmpi;
    
    //_PDBA("p347_pwr_ioctl start: inode=0x%p, cmd=%d, arg=0x%p\n",(void*)inode,cmd,(void*)arg);
    switch (cmd) {
	case p347_HWMON_IOCTL_INITREAD_4030: {
	
	break; }
	case p347_HWMON_IOCTL_GET_4030: {
	
	break; }
	//-------------------------------------------------------------------------TEMPERATURE SENSOR
	case p347_HWMON_IOCTL_READ_TEMP_SENSOR: {
	    __raw_writel(0x200,scm_base_addr+0x2524);
	    msleep(1);
	    reg_val = __raw_readl(scm_base_addr+0x2524);
	    //_PDBA("TEMP_SENSOR VAL READ = %4x\n",reg_val);
	    copy_to_user((void*)arg,&reg_val,4);
	    __raw_writel(0x000,scm_base_addr+0x2524);
	break; }
	//-------------------------------------------------------------------------BEEPER
	case p347_HWMON_IOCTL_DO_BEEP: {
	    if (arg != 0) {
		beep_work.len_ms = (arg & 0xFFF);
		//printk("planning to beep %d\n",beep_work.len_ms);
		queue_work(beep_workqueue, &beep_work.my_work);
	    } else return -HWMON_ERR_INVALID_PARAMS;
	break; }
	//-------------------------------------------------------------------------BATTERY MONITOR
	/*
	case p347_HWMON_IOCTL_BAT_UPDATE_BASEVAL: {
	    stc_write_base(arg & 0xFF);
	break; }
	*/
	case p347_HWMON_IOCTL_BAT_READ: {
	    if (arg != 0) {
	    	user_bat_data = (t_batmon_data*)arg;
	    	
	    	spin_lock_irqsave(&stc_data.lock,flags);
	    	ret = copy_to_user(user_bat_data,&stc_data.bat,sizeof(t_batmon_data));
	    	spin_unlock_irqrestore(&stc_data.lock,flags);
	    	
		if (ret != 0)
		    return -HWMON_ERR_COPY_BAT_DATA;
	    } else return -HWMON_ERR_INVALID_PARAMS;
	break; }
	case p347_HWMON_IOCTL_BAT_SETPARAMS: {
	    if (arg != 0) {
		user_bat_params = (t_batmon_params*)arg;
		
		//TODO: check parameters validity
		
		spin_lock_irqsave(&stc_data.lock,flags);
		ret = copy_from_user(&stc_data.bpar,user_bat_params,sizeof(t_batmon_params));
		spin_unlock_irqrestore(&stc_data.lock,flags);
		
		printk("------------------------------\n");
		printk("New battery params: \n");
		printk("Vmult = %d\n",stc_data.bpar.voltage_multiplier);
		printk("Max charge uAh = %d\n",stc_data.bpar.max_battery_charge_uAh);
		printk("Min charge uA = %d\n",stc_data.bpar.minimum_charge_current_uA);
		printk("------------------------------\n");
		printk("Min possible bat mV = %d\n",stc_data.bpar.minimum_possible_battery_voltage_mV);
		printk("Critical bat mV = %d\n",stc_data.bpar.critical_battery_voltage_mV);
		printk("Low bat mV = %d\n",stc_data.bpar.low_battery_voltage_mV);
		printk("Alarm bat mV = %d\n",stc_data.bpar.alarm_battery_voltage_mV);
		printk("Full bat mV = %d\n",stc_data.bpar.full_battery_voltage_mV);
		printk("Half bat mV = %d\n",stc_data.bpar.half_battery_voltage_mV);
		printk("------------------------------\n");
		printk("Bat alarm percent = %d\n",stc_data.bpar.b_alarm_percent);
		printk("Bat mincharge percent = %d\n",stc_data.bpar.b_mincharge_percent);
		printk("------------------------------\n");
		
		//ret = stc_write_base(stc_data.bpar.max_battery_charge_uAh/2);
		//printk("set_params stc_write_base %d ret %d\n",stc_data.bpar.max_battery_charge_uAh/2,ret);
		//printk("set voltage multiplier %d\n",stc_data.bpar.voltage_multiplier);
		if (ret != 0)
		    return -HWMON_ERR_COPY_BAT_DATA;
	    } else return -HWMON_ERR_INVALID_PARAMS;
	break; }
	case p347_HWMON_IOCTL_FORCE_CHARGER: {
	    if (arg == 0) TURN_CHARGER_OFF;
	    else TURN_CHARGER_ON;
	break; }
	case p347_HWMON_IOCTL_REESTIMATE: {
	    if (arg != 0) {
		copy_from_user(&mv_estimation[0],arg,101*sizeof(unsigned int));
		while (atomic_read(&bat_first_read) != 0) {
		    printk("waiting for first battery read...\n");
		    msleep(200);
		}
		spin_lock_irqsave(&stc_data.lock,flags);
		//mv_estimation = arg;
		stc_data.bat.charge_percent = 0;
		for (tmpi=100; tmpi>0; tmpi--) {
		    if (stc_data.bat.val_ucharge_mV >= mv_estimation[tmpi]) {
		        printk("readed %d mV is >= %d mv_estimation\n",stc_data.bat.val_ucharge_mV,mv_estimation[tmpi]);
		        stc_data.bat.charge_percent = tmpi-1;
			break;
		    }
		}
		
	        tmpi = stc_data.bpar.max_battery_charge_uAh * stc_data.bat.charge_percent;
		real_charge_uah = tmpi / 100;
		prev_charge_uah = stc_data.bat.val_current_charge_uAh;
		spin_unlock_irqrestore(&stc_data.lock,flags);
		printk("Re-estimate: charge %d percent, write %d to real charge uah\n",stc_data.bat.charge_percent,real_charge_uah);
	    } else
		return -EINVAL;
	break; }
	//--------------------------------------------------------------------------------DISPLAY BACKLIGHT
	case p347_HWMON_IOCTL_BACKLIGHT_SWITCH: {
	    if (arg == 0) { //switch off
		gpio_set_value(p347_GPIO_BACKLIGHT_POWER,p347_BACKLIGHT_OFF);
	    } else { //switch on
	        gpio_set_value(p347_GPIO_BACKLIGHT_POWER,p347_BACKLIGHT_ON);
	    }
	break; }
        default: return -HWMON_ERR_INVALID_IOCTL;
    };
    //_PDB("p347_pwr_ioctl end\n");
    return 0;
}

//==========================================================================

static const struct file_operations p347_pwr_fops = {
    .owner		= THIS_MODULE,
    .unlocked_ioctl	= p347_pwr_ioctl,
    .open		= p347_pwr_open,
    .release		= p347_pwr_release,
    .fasync		= NULL,
    .llseek 		= no_llseek,
};

static int __devinit stc_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    _PDB("stc_probe\n");
    stc_data.client = client;
    return 0;
}

static int __devexit stc_remove(struct i2c_client *client) {
    return 0;
}

static const struct i2c_device_id bcharge_id[] = {
    { "stc3105iqt", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bcharge_id);

//==========================================================================
#define read_and_print(_offset_,_name_) \
    reg_val = __raw_readl(prm_base + _offset_);\
    printk("%s : val = 0x%08lx\n",_name_,reg_val)

void print_prm_registers(void) {
    unsigned long reg_val = 0;
    unsigned long prm_base = (unsigned long)ioremap(0x48300000,SZ_4K);
    
    read_and_print(0x60E0,"IVA2_STATE");
    read_and_print(0x60E8,"IVA2_PREVS");
    read_and_print(0x69E0,"MPU_STATE");
    read_and_print(0x69E8,"MPU_PREVS");
    read_and_print(0x6AE0,"CORE_STATE");
    read_and_print(0x6AE8,"CORE_PREVS");
    read_and_print(0x6BE0,"SGX_STATE");
    read_and_print(0x6BE8,"SGX_PREVS");
    read_and_print(0x6EE0,"DSS_STATE");
    read_and_print(0x6EE8,"DSS_PREVS");
    read_and_print(0x6FE0,"CAM_STATE");
    read_and_print(0x6FE8,"CAM_PREVS");
    read_and_print(0x70E0,"PER_STATE");
    read_and_print(0x70E8,"PER_PREVS");
    read_and_print(0x73E0,"NEON_STATE");
    read_and_print(0x73E8,"NEON_PREVS");
    read_and_print(0x74E0,"USB_STATE");
    read_and_print(0x74E8,"USB_PREVS");
}

static int p347_hwmon_suspend(struct device* dev) {
    printk("p347_pwr suspend call\n");
    
    print_prm_registers();
    
    return 0;
}

static int p347_hwmon_resume(struct device* dev) {
    printk("p347_pwr resume call\n");
    
    print_prm_registers();
    
    return 0;
}

static const struct dev_pm_ops p347_hwmon_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(p347_hwmon_suspend,p347_hwmon_resume)
};

static struct i2c_driver stc_driver = {
    .driver = {
	.name	= "stc3105iqt",
	.owner	= THIS_MODULE,
	.pm	= &p347_hwmon_pm_ops,
    },
    .probe	= stc_probe,
    .remove	= __devexit_p(stc_remove),
    .id_table	= bcharge_id,
};

//==========================================================================
//==========================================================================
//==========================================================================

#define CHECK_VALUE(_reg_)\
    reg_val = 0;\
    ret = twl_i2c_read_u8(TWL4030_MODULE_MADC,&reg_val,_reg_);\
    _PDBA("ret read=%d, verify read=0x%02x\n",ret,reg_val)

#define CHECK_VALUE_E(_mod_,_reg_)\
    reg_val = 0;\
    ret = twl_i2c_read_u8(_mod_,&reg_val,_reg_);\
    _PDBA("ret read=%d, verify read=0x%02x\n",ret,reg_val)

static int __init p347_pwr_init(void)
{
    int ret;
    //u8 reg_val;
    unsigned long regl;
    unsigned long padconf_addr;
    unsigned long old;
    unsigned long new;
    //unsigned long rate;

    unsigned long gpio_base = (unsigned long)ioremap(0x48310000,SZ_4K); //GPIO bank 1
    scm_base_addr = (unsigned long)ioremap(0x48000000,SZ_4K);
    printk("p347_pwr_init\n");
    printk("p347 HWMON start: %s\n",p347_DRV_VERSION);
    spin_lock_init(&stc_data.lock);

    //clock enable (iclk) for GPIO1
    old = __raw_readl(scm_base_addr+0x4c10);
    printk("default CM_ICLKEN_WKUP register = 0x%lx\n",old);
    new = old | 0x8; //set bit3 (EN_GPIO1) to 1
    __raw_writel(new,scm_base_addr+0x4c10);
    new = __raw_readl(scm_base_addr+0x4c10);
    printk("new CM_ICLKEN_WKUP register = 0x%lx\n",new);
    
    //clock enable (fclk) for GPIO1
    old = __raw_readl(scm_base_addr+0x4c00);
    printk("default CM_FCLKEN_WKUP register = 0x%lx\n",old);
    new = old | 0x8; //set bit3 (EN_GPIO1) to 1
    __raw_writel(new,scm_base_addr+0x4c00);
    new = __raw_readl(scm_base_addr+0x4c00);
    printk("new CM_FCLKEN_WKUP register = 0x%lx\n",new);
    
    //print CM_IDLEST_WKUP
    old = __raw_readl(scm_base_addr+0x4c20);
    printk("read-only CM_IDLEST_WKUP register = 0x%lx\n",old);
    
    //autoidle register
    old = __raw_readl(scm_base_addr+0x4c30);
    printk("CM_AUTOIDLE_WKUP register = 0x%lx\n",old);
    new = old | 0x8; //set bit3 to 1
    __raw_writel(new,scm_base_addr+0x4c30);
    new = __raw_readl(scm_base_addr+0x4c30);
    printk("new CM_AUTOIDLE_WKUP register = 0x%lx\n",new);
    
    
    old = __raw_readl(gpio_base + 0x30); //GPIO_CTRL
    new = 0; //enable module, no gating
    __raw_writel(new,gpio_base + 0x30);
    new = __raw_readl(gpio_base + 0x30);
    printk("CTRL for GPIO old=0x%08lx, new=0x%08lx\n",old,new);
                    
    old = __raw_readl(gpio_base + 0x10); //GPIO_SYSCONFIG
    new = 0x8; //No-idle mode
    __raw_writel(new,gpio_base + 0x10);
    new = __raw_readl(gpio_base + 0x10);
    printk("SYSCONFIG for GPIO old=0x%08lx, new=0x%08lx\n",old,new);
    
    //gpio_13
    old = __raw_readl(gpio_base + 0x34); //GPIO_OE
    printk("OE for GPIO old=0x%08lx \n",old);
    /*
    new = old & ~(1 << 13); //set as output
    __raw_writel(new,gpio_base + 0x34);
    new = __raw_readl(gpio_base + 0x34);
    _PDBA("OE for GPIO old=0x%08x, new=0x%08x\n",old,new);
    */
    //-------------------------------------------------------------------enable Temperature sensors functional clock
    regl = __raw_readl(scm_base_addr+0x4a08);
    __raw_writel((regl|0x2),scm_base_addr+0x4a08);


/*
    _PDB("PROTECT_KEY\n");
    //CHECK_VALUE_E(TWL4030_MODULE_RTC,0x44);
    CHECK_VALUE_E(TWL4030_MODULE_PM_MASTER,TWL4030_PM_MASTER_PROTECT_KEY);
    twl_i2c_write_u8(TWL4030_MODULE_RTC,0xC0,TWL4030_PM_MASTER_PROTECT_KEY);
    twl_i2c_write_u8(TWL4030_MODULE_RTC,0x0C,TWL4030_PM_MASTER_PROTECT_KEY);
    CHECK_VALUE_E(TWL4030_MODULE_PM_MASTER,TWL4030_PM_MASTER_PROTECT_KEY);
    
    _PDB("BOOT_CFG\n");
    CHECK_VALUE_E(TWL4030_MODULE_PM_MASTER,TWL4030_PM_MASTER_CFG_BOOT);//BOOT_CFG
    //twl_i2c_write_u8(TWL4030_MODULE_BACKUP,0x0A,0x3B);
    //CHECK_VALUE_E(TWL4030_MODULE_BACKUP,0x3B);
    
    _PDB("GPBR1\n");
    CHECK_VALUE_E(TWL4030_MODULE_INTBR,0x0C);//GPBR1
    twl_i2c_write_u8(TWL4030_MODULE_INTBR,(reg_val|0x10),0x0C);
    CHECK_VALUE_E(TWL4030_MODULE_INTBR,0x0C);
    
    //Set up MADC registers (slave 2 adr 0x4a)
    
    _PDB("SW_IMR1\n");
    reg_val = 0x0D; //SW1_IMR1 interrupt unmask
    twl_i2c_write_u8(TWL4030_MODULE_MADC,reg_val,TWL4030_MADC_IMR1);
    CHECK_VALUE(TWL4030_MADC_IMR1);
    
    _PDB("SIH_CTRL\n");
    reg_val = 0x05;
    twl_i2c_write_u8(TWL4030_MODULE_MADC,reg_val,TWL4030_MADC_SIH_CTRL);
    CHECK_VALUE(TWL4030_MADC_SIH_CTRL);
    
    _PDB("CTRL1\n");
    reg_val = 0x01; //CTRL1 power up MADC
    twl_i2c_write_u8(TWL4030_MODULE_MADC,reg_val,0x00);
    CHECK_VALUE(0x00);

    _PDB("SW1SELECT_LSB\n");
    reg_val = 0x01; //SW1SELECT_LSB select ADCIN0
    //reg_val = 0x04; //bit2 = ADCIN2
    twl_i2c_write_u8(TWL4030_MODULE_MADC,reg_val,0x06);
    CHECK_VALUE(0x06);
    
    _PDB("SW1AVERAGE\n");
    //reg_val = 0x01; //SW1AVERAGE set for ADCIN0 (4 cycle averaging)
    //reg_val = 0x04;
    //twl_i2c_write_u8(TWL4030_MODULE_MADC,reg_val,0x08);
    CHECK_VALUE(0x08);
*/
    //-----------------------------------------------------------charger
    //-----------------------------------------------------------shutdown pin
    ret = gpio_request(p347_GPIO_CHARGER_SHDN,"gpio_charger_shdn");
    if (ret != 0) {
    	_PDBA("ERROR: cannot request gpio_charger_shdn, ret=%d\n",ret);
    	return ret;
    }

    padconf_addr = scm_base_addr + CHARGER_SHDN_PADCONF_ADDR;
    regl = __raw_readl(padconf_addr);
    if (CHARGER_SHDN_PADCONF_OFFSET) regl &= 0x0000FFFF; else regl &= 0xFFFF0000;
    regl |= (CHARGER_SHDN_PADCONF_VALUE << CHARGER_SHDN_PADCONF_OFFSET);
    __raw_writel(regl,padconf_addr);

    gpio_direction_output(p347_GPIO_CHARGER_SHDN,0); //SHDN by default
    //-----------------------------------------------------------fullchg pin
    ret = gpio_request(p347_GPIO_CHARGER_FULLCHG,"gpio_charger_fullchg");
    if (ret != 0) {
    	_PDBA("ERROR: cannot request gpio_charger_fullchg, ret=%d\n",ret);
    	return ret;
    }

    padconf_addr = scm_base_addr + CHARGER_FULLCHG_PADCONF_ADDR;
    regl = __raw_readl(padconf_addr);
    if (CHARGER_FULLCHG_PADCONF_OFFSET) regl &= 0x0000FFFF; else regl &= 0xFFFF0000;
    regl |= (CHARGER_FULLCHG_PADCONF_VALUE << CHARGER_FULLCHG_PADCONF_OFFSET);
    __raw_writel(regl,padconf_addr);

    gpio_direction_input(p347_GPIO_CHARGER_FULLCHG);
    //-----------------------------------------------------------fullchg pin
    ret = gpio_request(p347_GPIO_CHARGER_FAULT,"gpio_charger_fault");
    if (ret != 0) {
    	_PDBA("ERROR: cannot request gpio_charger_fault, ret=%d\n",ret);
    	return ret;
    }

    padconf_addr = scm_base_addr + CHARGER_FAULT_PADCONF_ADDR;
    regl = __raw_readl(padconf_addr);
    if (CHARGER_FAULT_PADCONF_OFFSET) regl &= 0x0000FFFF; else regl &= 0xFFFF0000;
    regl |= (CHARGER_FAULT_PADCONF_VALUE << CHARGER_FAULT_PADCONF_OFFSET);
    __raw_writel(regl,padconf_addr);

    gpio_direction_input(p347_GPIO_CHARGER_FAULT);
    //-----------------------------------------------------------fastchg pin
    ret = gpio_request(p347_GPIO_CHARGER_FASTCHG,"gpio_charger_fastchg");
    if (ret != 0) {
    	_PDBA("ERROR: cannot request gpio_charger_fastchg, ret=%d\n",ret);
    	return ret;
    }

    padconf_addr = scm_base_addr + CHARGER_FASTCHG_PADCONF_ADDR;
    regl = __raw_readl(padconf_addr);
    if (CHARGER_FASTCHG_PADCONF_OFFSET) regl &= 0x0000FFFF; else regl &= 0xFFFF0000;
    regl |= (CHARGER_FASTCHG_PADCONF_VALUE << CHARGER_FASTCHG_PADCONF_OFFSET);
    __raw_writel(regl,padconf_addr);

    gpio_direction_input(p347_GPIO_CHARGER_FASTCHG);
    /*
    //-----------------------------------------------------------backlight
    ret = gpio_request(p347_GPIO_BACKLIGHT_POWER,"gpio_backlight_power");
    if (ret != 0) {
	_PDBA("ERROR: cannot request gpio_backlight_power, ret=%d\n",ret);
	return ret;
    }
    gpio_direction_output(p347_GPIO_BACKLIGHT_POWER, p347_BACKLIGHT_ON);
    */
    //-----------------------------------------------------------beep
    spin_lock_init(&beep_work.lock);

    ret = gpio_request(p347_GPIO_BEEP,"gpio_beep");
    if (ret != 0) {
    	_PDBA("ERROR: cannot request gpio_beep, ret=%d\n",ret);
    	return ret;
    }

    padconf_addr = scm_base_addr + BEEP_PADCONF_ADDR;
    regl = __raw_readl(padconf_addr);
    regl &= 0x0000FFFF;
    regl |= (BEEP_PADCONF_VALUE << BEEP_PADCONF_OFFSET);
    __raw_writel(regl,padconf_addr);

    gpio_direction_output(p347_GPIO_BEEP,1);
    
    new = __raw_readl(gpio_base + 0x34);
    _PDBA("OE for GPIO new=0x%08lx\n",new);
    
    gpio_set_value(p347_GPIO_BEEP,0);//silence
    
    INIT_WORK(&beep_work.my_work, do_beep);
    beep_workqueue = create_singlethread_workqueue("bt_beep");

    //------------------------------------------------------------i2c - stc3105iqt
    ret = i2c_add_driver(&stc_driver);
    printk("i2c_add_driver for stc3105iqt ret %d\n",ret);

    //------------------------------------------------------------register common interface part
    ret = register_chrdev(p347_HWMON_CHARDEV_NUM,p347_HWMON_CHARDEV_NAME,&p347_pwr_fops);

    _PDB("p347_pwr_init end\n");
    
    //set default values
    stc_data.bat.is_bm_connected = 0;
    stc_data.bat.is_charging = 0;
    stc_data.bat.is_fault = 0;
    stc_data.bat.battery_presense = 0;
    stc_data.bat.outer_power_presense = 0;
    stc_data.bat.alarm_low_voltage = 0;
    stc_data.bat.no_connect_counter = 0;
    
    stc_data.bpar.voltage_multiplier = 4000;
    stc_data.bpar.max_battery_charge_uAh = 2200000;
    stc_data.bpar.low_battery_voltage_mV = 15700;
    stc_data.bpar.alarm_battery_voltage_mV = 14000;
    stc_data.bpar.full_battery_voltage_mV = 16700;
    stc_data.bpar.minimum_charge_current_uA = 20000;
    stc_data.bpar.critical_battery_voltage_mV = 12000;
    stc_data.bpar.minimum_possible_battery_voltage_mV = 9000;
    stc_data.bpar.half_battery_voltage_mV = 14000;
    
    TURN_CHARGER_OFF;
    msleep(1);
    TURN_CHARGER_ON;
    
    start_monitoring();
    
    return 0;
}
//module_initcall(p347_pwr_init);

//==========================================================================

static void __exit p347_pwr_exit(void)
{
    //int i;
    //_PDB("p347_pwr_exit start\n");
    
    i2c_del_driver(&stc_driver);
    
    unregister_chrdev(p347_HWMON_CHARDEV_NUM,p347_HWMON_CHARDEV_NAME);
    
    flush_workqueue(beep_workqueue);
    destroy_workqueue(beep_workqueue);
    
    gpio_free(p347_GPIO_BEEP);
    
    gpio_free(p347_GPIO_CHARGER_SHDN);
    gpio_free(p347_GPIO_CHARGER_FULLCHG);
    gpio_free(p347_GPIO_CHARGER_FAULT);
    gpio_free(p347_GPIO_CHARGER_FASTCHG);
    
    //gpio_free(p347_GPIO_BACKLIGHT_POWER);
    
    stop_monitoring();
    _PDB("p347_pwr_exit end\n");
}

//==========================================================================
//==========================================================================
//==========================================================================
module_init(p347_pwr_init);
module_exit(p347_pwr_exit);
//==========================================================================
MODULE_AUTHOR("Konstantin Utkin");
MODULE_LICENSE("GPL");
//==========================================================================
