#ifndef __GPS_H
#define __GPS_H

void gps_init(void);
void gps_bind(void(*handler)(struct gps *gps));

#endif
