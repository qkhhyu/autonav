#include <string.h>
#include <math.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "sail.h"
#include "gnss.h"
#include "svc_gnss.h"
#include "fifo.h"
#include "svc_sail.h"

#include "auto_boat_cruise.h"
#include "auto_pid.h"

#define FAST_SPEED	400
#define SLOW_SPEED	300
struct cruise_handler cruise_handler;

event_t  cruise_event;
mutex_t cruise_mutex;

long double currentspeed = 0;
int motor_speed = 100;
double set_boat_speed = BOAT_SPEED;

struct pid_t pid_speed;

//巡航控制，根据方位角
void cruise_gps_control(long double dist,int azimuth,int heading)
{
	int azimuth_difference = 0;
	int control_angle = 0;
	azimuth_difference = ((360-heading)+azimuth)%360;//计算两个方位角的差值
	#ifdef program1
	if(dist<3.0f)//距离较近，计算出的方位角可能误差较大，减速保持直行
	{
		cruise_handler.go_strgaight(FAST_SPEED,FAST_SPEED);
	}
	#endif
	#ifdef program0
	if(dist<1.0f)//距离较近，计算出的方位角可能误差较大，保持直行
	{
		cruise_handler.go_strgaight(FAST_SPEED,FAST_SPEED);
	}
	#endif
	else if(azimuth_difference<180 && azimuth_difference>10)//差值小于180，则向右转
	{
		control_angle = azimuth_difference;
		if(control_angle>90)//大角度，尽可能短时间内调整好自身方位角
		{
			cruise_handler.turn_right(FAST_SPEED,-SLOW_SPEED,90);
		}
		else if(control_angle > 10)
		{
			cruise_handler.turn_right(FAST_SPEED,SLOW_SPEED,control_angle);
		}
		else
		{
			cruise_handler.turn_right(FAST_SPEED,FAST_SPEED,5);
		}			
	}
	else if(azimuth_difference>180 && azimuth_difference<350)//不考虑等于180的情况，如果是等于180，也是左转
	{
		control_angle = 360-azimuth_difference;
		if(control_angle>90)//大角度，尽可能短时间内调整好自身方位角
		{
			cruise_handler.turn_left(-SLOW_SPEED,FAST_SPEED,90);
		}
		else if(control_angle>10)
		{
			cruise_handler.turn_left(SLOW_SPEED,FAST_SPEED,control_angle);
		}
		else
		{
			cruise_handler.turn_left(FAST_SPEED,FAST_SPEED,5);
		}
	}
	else//直行
	{
		cruise_handler.go_strgaight(FAST_SPEED,FAST_SPEED);
	}
		
}

void cruise_gps_control_pid(int16_t speed1,int16_t speed2,int rudderangle)
{
	mutex_lock(cruise_mutex);
	// cruise_handler.control(speed1,speed2,rudderangle);
	svc_sail_set_speed(speed1);
	svc_sail_set_dir(rudderangle);

	mutex_unlock(cruise_mutex);
}

void curise_general_control(int16_t speed1, int16_t speed2, int16_t dir)
{
	mutex_lock(cruise_mutex);
	cruise_handler.control(speed1,speed2,dir);
	mutex_unlock(cruise_mutex);
}


void curise_gps_control_pid_1(long double dist,int azimuth,int heading,int rudderangle)
{
	int azimuth_difference = 0;
	int control_angle = 0;
	azimuth_difference = ((360-heading)+azimuth)%360;//计算两个方位角的差值

	if(dist<3.0f)
	{
		cruise_handler.go_strgaight(SLOW_SPEED,SLOW_SPEED);
	}
	else if(azimuth_difference<180 && azimuth_difference>10)//差值小于180，则向右转
	{
		control_angle = azimuth_difference;
		if(control_angle>90)
		{
			cruise_handler.control(FAST_SPEED,-SLOW_SPEED,rudderangle);
		}
		else
		{
			cruise_handler.control(FAST_SPEED,FAST_SPEED,rudderangle);
		}	
	}
	else if(azimuth_difference>180 && azimuth_difference<350)//不考虑等于180的情况，如果是等于180，也是左转
	{
		control_angle = 360-azimuth_difference;
		if(control_angle>90)
		{
			cruise_handler.control(-SLOW_SPEED,FAST_SPEED,rudderangle);
		}
		else
		{
			cruise_handler.control(FAST_SPEED,FAST_SPEED,rudderangle);
		}
	}
	else
	{
		cruise_handler.control(FAST_SPEED,FAST_SPEED,rudderangle);
	}
}

void cruise_feed_control(int16_t speed1,int16_t speed2)
{
	cruise_handler.feed(speed1,speed2);
}


int cruise_dist_control(struct dist *dist)
{
	if(dist->front==0 && dist->right1==0 && dist->back==0)
	{
		return 0;
	}
	if(dist->front<2000 || dist->right1<1500 || dist->back<1500)
	{
		cruise_handler.turn_left(-SLOW_SPEED,FAST_SPEED,150);
	}
	if(dist->front<2500 || dist->right1<2000 || dist->back<2000)
	{
		cruise_handler.turn_left(-SLOW_SPEED,FAST_SPEED,90);
	}
	if(dist->front<3000 || dist->right1<2500 || dist->back<2500)
	{
		cruise_handler.turn_left(-SLOW_SPEED,FAST_SPEED,60);
	}
	return 1;
}

void cruise_bind(struct cruise_handler *handler)
{
	if(handler)
	{
		memcpy(&cruise_handler, handler, sizeof(struct cruise_handler));
	}
}

#define SPEEDFIFOSIZE	10
float speedtemp[SPEEDFIFOSIZE];
char speed_index = 0;
float speed_sum = 0.0f;

//速度均值滤波
float speed_filter(float speed)
{
//	char count = 0;
	float fir = 0.00f;
	
	fir = speedtemp[speed_index];
	speedtemp[speed_index++]  = speed;
	if(speed_index==SPEEDFIFOSIZE)
	{
		speed_index = 0;//先进先出
	}
	
	speed_sum = speed_sum+speed;
	speed_sum = speed_sum-fir;
//	for(count = 0;count<SPEEDFIFOSIZE;count++)
//	{
//		sum += speedtemp[count];
//	}
	return (speed_sum/SPEEDFIFOSIZE);
}

//获取设置当前位置经纬度坐标和速度
void auto_get_gpsspeed(struct gnss *gnss)
{
	mutex_lock(cruise_mutex);
	currentspeed = gnss->speed;
	currentspeed = currentspeed * 1.852f;//（海里/小时）转换成（千米/小时）
	
	currentspeed = speed_filter(currentspeed);
	mutex_unlock(cruise_mutex);	
	
//	event_post(cruise_event);
	static int div;
	if(++div > 10)
	{
		event_post(cruise_event);
		div = 0;
		LOG_HMI("speed=%fkm/h",currentspeed);
	}
	
}

void auto_cruise_pid_init(void)
{
	PID_struct_init(&pid_speed,Speed_pid,Vi_Position_Pid,1000,1000,appvar.pid_config[1][0],appvar.pid_config[1][1],appvar.pid_config[1][2]);
	pid_speed.f_pid_reset(&pid_speed,appvar.pid_config[1][0],appvar.pid_config[1][1],appvar.pid_config[1][2]);
}

//船调整线程，调整航行速度
static void auto_cruise_thread(void *arg)
{
	struct dist dist;
	LOG("[AUTO:CRUISE]init\r\n");
	
	double rout = 0;
	
	
	appvar.pid_config[1][0] = 3.5f;
	appvar.pid_config[1][1] = 0.035f;
	appvar.pid_config[1][2] = 0;
	auto_cruise_pid_init();
	set_boat_speed = BOAT_SPEED;
	
	while(1)
	{
		event_timed_wait(cruise_event, 1100);
		mutex_lock(cruise_mutex);

		pid_speed.f_pid_reset(&pid_speed,appvar.pid_config[1][0],appvar.pid_config[1][1],appvar.pid_config[1][2]);
		rout = pid_calc(&pid_speed,currentspeed*100.0f, set_boat_speed*100.0f);
		mutex_unlock(cruise_mutex);	
		//LOG_HMI("rout=%f",rout);
		mutex_lock(cruise_mutex);		
		motor_speed = (int)rout;
		if(set_boat_speed == BOAT_SPEED)
		{
			motor_speed = 100;
		}
		else
		{
			motor_speed = 70;
		}
		
//		motor_speed = appvar.dst_course;
//		LOG_HMI("mospeed=%d",motor_speed);
		mutex_unlock(cruise_mutex);			
	}
}

void auto_cruise_init(void)
{
	//svc_gps_bind(auto_get_gpsspeed);
	cruise_event = event_create();
	cruise_mutex = mutex_create();
	thread_create(auto_cruise_thread,0,10240);
}

