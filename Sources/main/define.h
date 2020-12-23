#ifndef __DEFINE_H
#define __DEFINE_H
#include <stdint.h>

//系统时间
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

//定位数据
struct gps
{
	uint8_t locked;       //定位成功
	double  longitude;    //经度
	double  latitude;     //纬度
	float   speed;        //速度
	float   course;       //航向
};

//测距状态
struct dist
{
	uint16_t front;       //前
	uint16_t back;        //后
	uint16_t right1;      //右前
	uint16_t right2;      //右后
};

//电池状态
struct batt
{
	uint16_t voltage;     //电压
	int8_t   temp;        //温度
	int8_t   state;       //状态
};

//电机状态
struct motor
{
	int16_t  speed;      //转速
	int8_t   temp;       //温度
	int8_t   reserved;   //保留
};

//三轴数据
struct axis
{
	int16_t x;
	int16_t y;
	int16_t z;
};

#endif
