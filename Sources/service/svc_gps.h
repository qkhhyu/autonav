#ifndef __SVC_GPS_H
#define __SVC_GPS_H

void svc_gps_init(void);
void svc_gps_bind(void(*handler)(struct gps *gps));

#endif
