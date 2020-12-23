#ifndef __RTC_H
#define __RTC_H

void rtc_init(void);
uint8_t rtc_get_date(uint8_t* year, uint8_t* month, uint8_t* day);
uint8_t rtc_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second);
uint8_t rtc_set_date(uint8_t year, uint8_t month, uint8_t day);
uint8_t rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second);

#endif
