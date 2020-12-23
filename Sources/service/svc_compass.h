#ifndef __SVC_COMPASS_H
#define __SVC_COMPASS_H

void svc_compass_init(void);
void svc_compass_bind(void(*handler)(int heading));
void svc_compass_input(int16_t mdm[3], int16_t acc[3]);
void svc_compass_set_cali(int ctl);
void svc_compass_cal_zero(void);

#endif
