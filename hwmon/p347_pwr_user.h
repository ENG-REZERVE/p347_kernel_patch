#ifndef P347_PWR_USER_H
#define P347_PWR_USER_H

#include "p347_err.h"

/*
 * 09.07.2014 - Added re-estimation with special curve

*/

#define p347_HWMON_CHARDEV_NAME				"p347_pwr_device"
#define p347_HWMON_CHARDEV_NUM				242

//TPS CALLS
#define p347_HWMON_IOCTL_INITREAD_4030		01
#define p347_HWMON_IOCTL_GET_4030			02

//TEMPERATURE SENSOR CALLS
#define p347_HWMON_IOCTL_READ_TEMP_SENSOR	10

//BEEPER CALLS
#define p347_HWMON_IOCTL_DO_BEEP			20

//BATTERY MONITOR CALLS
#define p347_HWMON_IOCTL_BAT_UPDATE_BASEVAL	30
#define p347_HWMON_IOCTL_BAT_READ			31
#define p347_HWMON_IOCTL_BAT_SETPARAMS		32
#define p347_HWMON_IOCTL_FORCE_CHARGER		33
#define p347_HWMON_IOCTL_REESTIMATE			34

//DSIPLAY BACKLIGHT
#define p347_HWMON_IOCTL_BACKLIGHT_SWITCH	40

#define p347_BACKLIGHT_ON					1
#define p347_BACKLIGHT_OFF					0

typedef struct {
    //------------------------------------------//Information flags
    unsigned char	is_bm_connected;	//Battery monitor connection
    unsigned char 	is_charging;        //FULLCHG or FASTCHG low
    unsigned char	is_fault;			//FAULT pin low
    unsigned char	shdn_state;			//state of ~shutdown pin
    unsigned char	battery_presense;	//Battery presense
    unsigned char	outer_power_presense;	//Outer power supply detected
    unsigned char       alarm_low_voltage;      //Battery low voltage alarm
    //------------------------------------------//Battery monitor data	
    unsigned short	no_connect_counter;
    signed int		val_base_charge_uAh;
    signed int		val_current_charge_uAh;
    signed int		val_icharge_uA;
    unsigned int	val_ucharge_mV;
    unsigned char	fullchg_bit;
    unsigned char	fastchg_bit;
    unsigned short	hex_current_charge;
    //------------------------------------------//Forecast
    unsigned char	charge_percent;		//0..100
    unsigned short	minutes_left;		//worktime_forecast
}t_batmon_data;

typedef struct {
    //------------------------------------------//Very important battery parameters
    unsigned short	voltage_multiplier;     		//Used for battery set (4000x by default)
    signed int		max_battery_charge_uAh; 		//Nominal battery charge capacity
    signed int		minimum_possible_battery_voltage_mV;	//Voltage limit for battery presense detection
    signed int		critical_battery_voltage_mV;		//Critical minimum of battery voltage for turning system OFF
    //------------------------------------------
    signed int		low_battery_voltage_mV; //When U<low_battery_voltage_mV, we should turn charger ON
    signed int		alarm_battery_voltage_mV;
    signed int		full_battery_voltage_mV;	//Set charge_base=max when U>=full
    signed int		minimum_charge_current_uA;	//Shutdown charger when I<minimum and U>=full
    signed int		half_battery_voltage_mV;
    //------------------------------------------
    signed int		b_alarm_percent;		//Turn alarm ON when % is low than this value
    signed int		b_mincharge_percent;		//Turn charger ON when % is low than this value
}t_batmon_params;

#endif
