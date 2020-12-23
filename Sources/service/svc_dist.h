#ifndef __SVC_DIST_H
#define __SVC_DIST_H

void svc_dist_init(void);
void svc_dist_bind(void(*handler)(struct dist *dist));
void svc_dist_start(void);
void svc_dist_stop(void);

#endif
