#ifndef __AUTO_LOCATION_H
#define __AUTO_LOCATION_H

extern int location_thread_switch;


void auto_location_set_destination(long double latitude,long double longitude);
void auto_location_resume(void);
void auto_location_init(void);

#endif
