#ifndef __TIME_H
#define __TIME_H

void time_init(void);
void time_get_date(struct date *date);
void time_set_date(struct date *date);
void time_get_timestamp(uint32_t *timestamp);
void time_set_timestamp(uint32_t timestamp);
void date_to_timestamp(struct date *date, uint32_t *timestamp);
void timestamp_to_date(uint32_t timestamp, struct date *date);

#endif
