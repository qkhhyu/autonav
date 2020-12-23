#ifndef __DEFINE_H
#define __DEFINE_H
#include <stdint.h>

//ϵͳʱ��
struct date
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
};

struct sys
{
	int rssi;
};

//��λ����
struct gps
{
	uint8_t locked;       //��λ�ɹ�
	double  longitude;    //����
	double  latitude;     //γ��
	float   speed;        //�ٶ�
	float   course;       //����
};

//���״̬
struct dist
{
	uint16_t front;       //ǰ
	uint16_t back;        //��
	uint16_t right1;      //��ǰ
	uint16_t right2;      //�Һ�
};

//���״̬
struct batt
{
	uint16_t voltage;     //��ѹ
	int8_t   temp;        //�¶�
	int8_t   state;       //״̬
};

//���״̬
struct motor
{
	int16_t  speed;      //ת��
	int8_t   temp;       //�¶�
	int8_t   reserved;   //����
};

//��������
struct axis
{
	int16_t x;
	int16_t y;
	int16_t z;
};

#endif
