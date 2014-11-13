#ifndef P347_PWR_H
#define P347_PWR_H

//Registers of STC3105IQT

//mode					RW
#define BATMON_REG_MODE			0
#define MODE_PWR_SAVE			0x04
#define MODE_ALM_ENA			0x08
#define MODE_GG_RUN			0x10

//control and status			RW
#define BATMON_REG_CTRL			1
#define CTRL_IO0DATA			0x01
#define CTRL_GG_RST			0x02
#define CTRL_GG_EOC			0x04
#define CTRL_VM_EOC			0x08
#define CTRL_PORDET			0x10
#define CTRL_ALM_SOC			0x20
#define CTRL_ALM_VOLT			0x40

//state-of-charge bits 0-7, 		RO
#define BATMON_REG_CHARGE_LOW		2
//state-of-charge bits 8-15		RO
#define BATMON_REG_CHARGE_HIGH 		3
//number of conversions bits 0-7, 	RO
#define BATMON_REG_COUNTER_LOW		4
//number of conversions bits 8-15	RO
#define BATMON_REG_COUNTER_HIGH		5
//battery current bits 0-7, 		RO
#define BATMON_REG_CURRENT_LOW		6
//battery current bits 8-15		RO
#define BATMON_REG_CURRENT_HIGH		7
//battery voltage bits 0-7, 		RO
#define BATMON_REG_VOLTAGE_LOW		8
//battery voltage bits 8-15		RO
#define BATMON_REG_VOLTAGE_HIGH		9
//state-of-charge base value bits 0-7, 	RW
#define BATMON_REG_SOC_BASE_LOW		10
//state-of-charge base value bits 8-15	RW
#define BATMON_REG_SOC_BASE_HIGH	11
//state-of-charge alarm level bits 0-7, RW
#define BATMON_REG_SOC_ALARM_LOW	12
//state-of-charge alarm level bits 8-15	RW
#define BATMON_REG_SOC_ALARM_HIGH	13
//Battery low voltage alarm level	RW
#define BATMON_REG_ALARM_VOLTAGE	14
//current threshold for the voltage relaxation counter		RW
#define BATMON_REG_CURRENT_THRES				15
//voltage relaxation counter		RO
#define BATMON_REG_RELAX_COUNT		16
//device ID register			RO
#define BATMON_REG_ID			24
//RAM registers for gas gauge           RW
#define BATMON_RAM_START		32
#define BATMON_RAM_SIZE			16

//TODO: access functions for STC3105IQT

//measurement bits value, value*100
//charge data: 2's complement, bit=6.70 uVh
#define BATMON_BITVAL_CHARGE		670
//battery current: 2's complement, bit=11.77 uV //maybe uA ???
#define BATMON_BITVAL_CURRENT		1177
//battery voltage: binary, bit=2.44 mV
#define BATMON_BITVAL_VOLTAGE		244

//sense resistor value*100 (50 mOm)
#define SENSE_RESISTOR_VALUE		5

//recounting macro: uAh = bits*step_x100/rsense_x100
#define uVh_bits_to_uAh(bits) 	((signed short)bits*BATMON_BITVAL_CHARGE)/SENSE_RESISTOR_VALUE
#define uAh_to_bits(uAh)	(uAh*SENSE_RESISTOR_VALUE)/BATMON_BITVAL_CHARGE

#define bits_to_mV(bits)	(bits*BATMON_BITVAL_VOLTAGE)/100

#define bits_to_uA(bits)	((signed short)bits*BATMON_BITVAL_CURRENT)/SENSE_RESISTOR_VALUE

//factory-default value for ID register
#define STC_PART_TYPE_ID		0x12


//----------------------------------------------GPIO DEFINITIONS
//output beep pin (active low)
#define p347_GPIO_BEEP				13
#define BEEP_PADCONF_ADDR			0x25D8
#define BEEP_PADCONF_OFFSET			16
#define BEEP_PADCONF_VALUE			0x0004

//output charger ~SHDN pin (active low)
#define p347_GPIO_CHARGER_SHDN			97
#define CHARGER_SHDN_PADCONF_ADDR		0x2110
#define CHARGER_SHDN_PADCONF_OFFSET		16
#define CHARGER_SHDN_PADCONF_VALUE		0x0004

//input charger ~FULLCHG pin (active low)
#define p347_GPIO_CHARGER_FULLCHG		101
#define CHARGER_FULLCHG_PADCONF_ADDR		0x2118
#define CHARGER_FULLCHG_PADCONF_OFFSET		16
#define CHARGER_FULLCHG_PADCONF_VALUE		0x011C

//input charger ~FAULT pin (active low)
#define p347_GPIO_CHARGER_FAULT			102
#define CHARGER_FAULT_PADCONF_ADDR		0x211C
#define CHARGER_FAULT_PADCONF_OFFSET		0
#define CHARGER_FAULT_PADCONF_VALUE		0x011C

//input charger ~FASTCHG pin (active low)
#define p347_GPIO_CHARGER_FASTCHG		103
#define CHARGER_FASTCHG_PADCONF_ADDR		0x211C
#define CHARGER_FASTCHG_PADCONF_OFFSET		16
#define CHARGER_FASTCHG_PADCONF_VALUE		0x011C

/*
#define p347_DISPATCH_GPIO			12
#define PADCONF_DISPATCH_ADDR			0x25d8
#define PADCONF_DISPATCH_OFFSET			0
#define PADCONF_DISPATCH_VALUE			0x0004
*/

#define p347_GPIO_BACKLIGHT_POWER		138
#define PADCONF_BACKLIGHT_ADDR			0x2168
#define PADCONF_BACKLIGHT_OFFSET		0
#define PADCONF_BACKLIGHT_VALUE			0x0004

//----------------------------------------------------------------------------------

#define MAX_NO_CONNECT_COUNTER		10

#include "p347_pwr_user.h"

#endif
