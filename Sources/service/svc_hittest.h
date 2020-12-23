#ifndef __HITTEST_H
#define __HITTEST_H

#define HIT_EVENT_FRONT  0x01 //Í·²¿×²»÷
#define HIT_EVENT_BACK   0x10 //Î²²¿×²»÷
#define HIT_EVENT_LEFT   0x02 //×ó²à×²»÷
#define HIT_EVENT_RIGHT  0x20 //ÓÒ²à×²»÷
#define HIT_EVENT_TOP    0x04 //¶¥²¿×²»÷
#define HIT_EVENT_BOTTOM 0x40 //µ×²¿×²»÷

void svc_hittest_init(void);
void svc_hittest_bind(void(*handler)(uint8_t evt));
void svc_hittest_input(int16_t data[3]);

#endif
