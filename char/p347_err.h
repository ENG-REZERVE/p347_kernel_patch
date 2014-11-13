#ifndef _p347_ERROR_HOLDER_
#define _p347_ERROR_HOLDER_

//коды ошибок 1..34 содержатся в errno-base.h

//коды ошибок 35.133 содержатся в errno.h

//Остальные коды являются application-specific


//Коды ошибок, возникающих в драйвере p347_fpga

#define p347_ERROR_SPI_INVAL_OPERATION				300
#define p347_ERROR_SPI_INVAL_ADC_NUM				301
#define p347_ERROR_SPI_INVAL_REG_NUM				302
#define p347_ERROR_SPI_INVAL_ARG					303

#define p347_ERROR_SYSTEM_INVAL_PID					400
#define p347_ERROR_SYSTEM_INVAL_FUN					401
#define p347_ERROR_SYSTEM_INVAL_SETUP				402
#define p347_ERROR_SYSTEM_DEVICE_BUSY				403
#define p347_ERROR_SYSTEM_CHANNEL_BUSY				404
#define p347_ERROR_SYSTEM_CHANNEL_NOT_CONFIGURED	405

#define p347_ERROR_VERSION_NOT_CHECKED				410
#define p347_ERROR_VERSION_INCOMPATIBLE				411

#define p347_ERROR_ADC_CHANNEL_BROKEN				500
#define p347_ERROR_ADC_BUFFER_OVERFLOW				501
#define p347_ERROR_ADC_LOAD_COEFS					502

#define p347_ERROR_FPGA_CRC_FIRMWARE        		600
#define p347_ERROR_FPGA_NOT_CONFIGURED      		601

//Outer
#define p347_ERROR_CHANNEL_NOT_AVAILABLE			700


//Ошибки драйвера p347_pwr

#define HWMON_ERR_INVALID_PARAMS					350
#define HWMON_ERR_INVALID_IOCTL						351
#define HWMON_ERR_COPY_BAT_DATA						352

//ошибки dsp_helper

#define DH_ERR_DSP_SETUP							800
#define DH_ERR_NOTIFY_FAILED						801
#define DH_ERR_RESPONCE_TIMEOUT						802
#define DH_ERR_STOP_FAILED							803

//Коды собщений alarmError, возникают асинхронно в DevHelper

#define ALARM_ERROR_NO_DRIVER						2001
#define ALARM_ERROR_DRIVER_BUFFER_OVERFLOW			2002
#define ALARM_ERROR_DSP_FAULT						2003
#define ALARM_ERROR_DATA_TIMEOUT					2004







/*
Первый список содержит коды причин завершения
программы для тех, кто вызывает её из себя путём форкования,
либо из скриптов шелла.
Используется программами: htest, srv_test
*/

//Благополучное нормальное завершение
#define END_REASON_OK					0
//Неверное количество параметров на входе htest2
#define END_REASON_INVALID_PARAMETERS_COUNT		1
//Выполнение прервано пользователем
#define END_REASON_ABORTED_BY_USER			2
//Не открывается ини-файл htest2
#define END_REASON_NO_INI_FILE				3
//Возник таймаут по получению данных на каком-то из каналов
#define END_REASON_CHANNEL_TIMEOUT			4
#define END_REASON_NO_MEMORY				5

//Индексы 10..20 - для кодов ошибок хелпера
//(к ним прибавляется 10 для передачи в _exit()

//Неклассифицированная ошибка (но известная)
#define END_REASON_UNCLASSIFIED				25

/*
Нижеследующий содержит индексы кодов ошибок в массиве ошибок.
Такой массив присущ каждому каналу и хелперу, при этом
элемент массива содержит подробный код ошибки.

По факту,в итоге эти коды наружу так и не выходят, служа в основном
внутренним ориентиром для хелпера.
*/

//------------------------------------------------Канальные индексы
//Количество
#define ERR_IDX_CH_MAX			20

//Ошибка открытия файла для записи данных
#define EH_CH_FILE			0
//Ошибка копирования данных из драйвера (пользовательские + COPY_ADC, COPY_TIME)
#define EH_CH_TRANSFER			1
//Переполнение буфера данных в классе канала
#define EH_CH_OVERFLOW			2
//Ошибка открытия файла для записи оборотных меток
#define EH_ACH_ROTFILE			3
//Переполнение буфера оборотных меток
#define EH_ACH_ROT_OVERFLOW		4
//Ошибка записи показаний АЦП в файл
#define EH_ACH_FWRITE_DAT		5
//Ошибка записи оборотных меток в файл
#define EH_ACH_FWRITE_ROT		6
//Ошибка настройки канала
#define EH_ACH_SETUP_CHANNEL		7
//Ошибка старта канала АЦП
#define EH_ACH_START_CHANNEL		8
//Ошибка стопа канала АЦП
#define EH_ACH_STOP_CHANNEL		9
//Ошибка настройки канала оборотов
#define EH_TCH_SETUP_CHANNEL		10
//Ошибка старта канала оборотов
#define EH_TCH_START_CHANNEL		11
//Ошибка стопа канала оборотов
#define EH_TCH_STOP_CHANNEL		12
//Ошибка записи временных отсчётов в файл
#define EH_TCH_FWRITE_TIME		13
//Ошибка старта канала оборотов
#define EH_TCH_START_ROT		14
//Ошибка стопа канала оборотов
#define EH_TCH_STOP_ROT			15
//Таймаут по получению данных по каналу
#define EH_CH_TIMEOUT			16
//Ошибка выполнения обработки на DSP
#define EH_CH_DSP_WORK			17

//------------------------------------------------Индексы хелпера
//Количество
#define ERR_IDX_HELP_MAX		10

//Ошибка проверки наличия данных для копирования
#define EH_HELP_CHECK_DATA		0
//Ошибка открытия символьного файла устройства
#define EH_HELP_DEV_OPEN		1
//Ошибка синхронного старта
#define EH_HELP_START_SYNC		2
//Ошибка синхронного стопа
#define EH_HELP_STOP_SYNC		3
//
#define EH_HELP_SPI_SET_HZ		4
//
#define EH_HELP_DATA_LOAD		5
//
#define EH_HELP_DSP_INIT		6

/*
Ошибки хелпера, как правило, отражают результат выполнения
соответствующих системных вызовов ioctl.
Для тех вызовов, которые относятся к конкретным каналам,
хелпер записывает ошибку в канальный массив.
*/

#endif
