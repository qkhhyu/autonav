/*
* RTCÊ±¼äÇý¶¯
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
*/
#include "stm32f4xx.h"
#include "rtc.h"

void rtc_init(void)
{
	RTC_InitTypeDef init;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);

	RCC_LSEConfig(RCC_LSE_OFF);
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY));
	RCC_LSEConfig(RCC_LSE_ON);
	while(!RCC_GetFlagStatus(RCC_FLAG_LSERDY));
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RCC_RTCCLKCmd(ENABLE);
	
	init.RTC_HourFormat = RTC_HourFormat_24;
	init.RTC_AsynchPrediv = 0x7F;
	init.RTC_SynchPrediv  = 0xFF;
	RTC_Init(&init);
}

uint8_t rtc_get_date(uint8_t *year, uint8_t *month, uint8_t *day)
{
	RTC_DateTypeDef rtc_date;
	RTC_GetDate(RTC_Format_BIN, &rtc_date);
	*year  = rtc_date.RTC_Year;
	*month = rtc_date.RTC_Month;
	*day   = rtc_date.RTC_Date;
	return 1;
}

uint8_t rtc_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second)
{
	RTC_TimeTypeDef rtc_time;
	RTC_GetTime(RTC_Format_BIN, &rtc_time);
	*hour  = rtc_time.RTC_Hours;
	*minute= rtc_time.RTC_Minutes;
	*second= rtc_time.RTC_Seconds;
	return 1;
}

uint8_t rtc_set_date(uint8_t year, uint8_t month, uint8_t day)
{
	RTC_DateTypeDef rtc_date;
	rtc_date.RTC_Year = year;
	rtc_date.RTC_Month= month;
	rtc_date.RTC_Date = day;
	rtc_date.RTC_WeekDay = 0;
	RTC_WaitForSynchro();
	RTC_SetDate(RTC_Format_BIN, &rtc_date);
	RTC_WaitForSynchro();
	return 1;
}

uint8_t rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second)
{
	RTC_TimeTypeDef rtc_time;
	rtc_time.RTC_Hours = hour;
	rtc_time.RTC_Minutes = minute;
	rtc_time.RTC_Seconds = second;
	
	RTC_WaitForSynchro();
	RTC_SetTime(RTC_Format_BIN, &rtc_time);
	RTC_WaitForSynchro();
	return 1;
}
