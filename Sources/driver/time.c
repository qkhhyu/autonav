/*
* Ê±¼äÇý¶¯
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
*/
#include <time.h>
#include "define.h"
#include "bsp.h"
#include "rtc.h"

void time_init(void)
{
	rtc_init();
}

void time_get_date(struct date *date)
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	
	rtc_get_time(&hour,&minute,&second);
	rtc_get_date(&year,&month,&day);

	date->year  = year + 2000;
	date->month = month;
	date->day   = day;
	date->hour  = hour;
	date->minute= minute;
	date->second= second;
}

void time_set_date(struct date *date)
{
	rtc_set_date(date->year-2000,date->month,date->day);
	rtc_set_time(date->hour,date->minute,date->second);
}

void time_get_timestamp(uint32_t *timestamp)
{
	struct date date;
	time_get_date(&date);
	date_to_timestamp(&date, timestamp);
}

void time_set_timestamp(uint32_t timestamp)
{
	struct date date;
	timestamp_to_date(timestamp, &date);
	time_set_date(&date);
}

void date_to_timestamp(struct date *date, uint32_t *timestamp)
{
	time_t times;
	struct tm tm;
	tm.tm_year = date->year - 1900;
	tm.tm_mon  = date->month - 1;
	tm.tm_mday = date->day;
	tm.tm_hour = date->hour;
	tm.tm_min  = date->minute;
	tm.tm_sec  = date->second;
	times = mktime(&tm);
	*timestamp = times - 28800;
}

void timestamp_to_date(uint32_t timestamp, struct date *date)
{
	time_t times;
	struct tm *tm;
	times = timestamp + 28800;
	tm = gmtime(&times);
	date->year   = tm->tm_year + 1900;
	date->month  = tm->tm_mon + 1;
	date->day    = tm->tm_mday;
	date->hour   = tm->tm_hour;
	date->minute = tm->tm_min;
	date->second = tm->tm_sec;	
}
