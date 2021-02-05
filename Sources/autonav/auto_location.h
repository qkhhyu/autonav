#ifndef __AUTO_LOCATION_H
#define __AUTO_LOCATION_H

extern int location_thread_switch;

enum
{
	STATE_MACHINE_MANUALCTRL = 0,
	STATE_AUTO_SAILING = 1,
    STATE_AUTO_PARKED = 2,
	STATE_DEBUGGING = 3,
};

void auto_location_set_destination(long double latitude,long double longitude);
void auto_location_resume(void);
void auto_location_init(void);

#endif
