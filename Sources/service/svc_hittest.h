#ifndef __HITTEST_H
#define __HITTEST_H

#define HIT_EVENT_FRONT  0x01 //ͷ��ײ��
#define HIT_EVENT_BACK   0x10 //β��ײ��
#define HIT_EVENT_LEFT   0x02 //���ײ��
#define HIT_EVENT_RIGHT  0x20 //�Ҳ�ײ��
#define HIT_EVENT_TOP    0x04 //����ײ��
#define HIT_EVENT_BOTTOM 0x40 //�ײ�ײ��

void svc_hittest_init(void);
void svc_hittest_bind(void(*handler)(uint8_t evt));
void svc_hittest_input(int16_t data[3]);

#endif
