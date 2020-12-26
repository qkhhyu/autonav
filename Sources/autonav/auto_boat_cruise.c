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

#include "auto_boat_cruise.h"
#include "auto_pid.h"

#define FAST_SPEED	400
#define SLOW_SPEED	300
struct cruise_handler cruise_handler;

event_t  cruise_event;
mutex_t cruise_mutex;

PID_Type pid_speed;
long double currentspeed = 0;
int motor_speed = 300;
double set_boat_speed = BOAT_SPEED;

struct pid_add pid_addspeed;

//Ѳ�����ƣ����ݷ�λ��
void cruise_gps_control(long double dist,int azimuth,int heading)
{
	int azimuth_difference = 0;
	int control_angle = 0;
	azimuth_difference = ((360-heading)+azimuth)%360;//����������λ�ǵĲ�ֵ
	#ifdef program1
	if(dist<3.0f)//����Ͻ���������ķ�λ�ǿ������ϴ󣬼��ٱ���ֱ��
	{
		cruise_handler.go_strgaight(FAST_SPEED,FAST_SPEED);
	}
	#endif
	#ifdef program0
	if(dist<1.0f)//����Ͻ���������ķ�λ�ǿ������ϴ󣬱���ֱ��
	{
		cruise_handler.go_strgaight(FAST_SPEED,FAST_SPEED);
	}
	#endif
	else if(azimuth_difference<180 && azimuth_difference>10)//��ֵС��180��������ת
	{
		control_angle = azimuth_difference;
		if(control_angle>90)//��Ƕȣ������ܶ�ʱ���ڵ���������λ��
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
	else if(azimuth_difference>180 && azimuth_difference<350)//�����ǵ���180�����������ǵ���180��Ҳ����ת
	{
		control_angle = 360-azimuth_difference;
		if(control_angle>90)//��Ƕȣ������ܶ�ʱ���ڵ���������λ��
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
	else//ֱ��
	{
		cruise_handler.go_strgaight(FAST_SPEED,FAST_SPEED);
	}
		
}

void cruise_gps_control_pid(long double dist,int azimuth,int heading,int rudderangle)
{
	mutex_lock(cruise_mutex);
	cruise_handler.control(motor_speed,motor_speed,rudderangle);
	mutex_unlock(cruise_mutex);
}

void curise_gps_control_pid_1(long double dist,int azimuth,int heading,int rudderangle)
{
	int azimuth_difference = 0;
	int control_angle = 0;
	azimuth_difference = ((360-heading)+azimuth)%360;//����������λ�ǵĲ�ֵ

	if(dist<3.0f)
	{
		cruise_handler.go_strgaight(SLOW_SPEED,SLOW_SPEED);
	}
	else if(azimuth_difference<180 && azimuth_difference>10)//��ֵС��180��������ת
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
	else if(azimuth_difference>180 && azimuth_difference<350)//�����ǵ���180�����������ǵ���180��Ҳ����ת
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
	if(dist->front==0 || dist->right1==0 || dist->right2==0)
	{
		return 0;
	}
	if(dist->front<3500 || dist->right1<2500 || dist->right2<2500)
	{
		cruise_handler.turn_left(-SLOW_SPEED,FAST_SPEED,90);
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

//��ȡ���õ�ǰλ�þ�γ��������ٶ�
void auto_get_gpsspeed(struct gnss *gnss)
{
	mutex_lock(cruise_mutex);
	currentspeed = gnss->speed;
	mutex_unlock(cruise_mutex);	
	event_post(cruise_event);
	static int div;
	if(++div > 10)
	{
		div = 0;
		LOG_HMI("speed=%f",currentspeed);
	}
	
}

void auto_cruise_pid_init(void)
{
	PID_init1(&pid_addspeed);
//	pid_init(&pid_speed);
//	pid_speed.Kp = 5.0f;
//	pid_speed.Ki = 0.0f;
//	pid_speed.Kd = 0.3f;
	pid_addspeed.Kp = appvar.pid_config[1][0];
	pid_addspeed.Ki = appvar.pid_config[1][1];
	pid_addspeed.Kd = appvar.pid_config[1][2];
	pid_addspeed.scope = 1000;
}

//�������̣߳����������ٶ�
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
		event_timed_wait(cruise_event, 100);
//		pid_speed.setaim = 1.1f*100;
		pid_addspeed.SetSpeed = set_boat_speed*100.0f;
		mutex_lock(cruise_mutex);
//		pid_speed.real_out = currentspeed*100.0f;
		pid_addspeed.ActualSpeed = currentspeed*100.0f;
		mutex_unlock(cruise_mutex);	
//		rout = pid_pos(&pid_speed);
		rout = PID_realize(&pid_addspeed,1);
		//LOG_HMI("rout=%f",rout);
		mutex_lock(cruise_mutex);		
		motor_speed = (int)rout;
		if(set_boat_speed == BOAT_SPEED)
		{
			motor_speed = 650;
		}
		else
		{
			motor_speed = 650;
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

