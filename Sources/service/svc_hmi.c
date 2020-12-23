/*
* 人机接口服务
* 蒋晓岗<kerndev@foxmail.com>
* 2019.12.27
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "svc.h"
#include "hmi.h"
#include "sail.h"
#include "feed.h"
#include "track.h"
#include "binconv.h"

//-------------------------------------------------------------------------------------------------
//回调函数
static void on_hid_connect(uint8_t state)
{
	LOG("on_hid_connect state=%02X\r\n", state);
	LOG_HMI("welcome!\r\n");
}
extern int motor_speed;
extern int location_thread_switch;
static int on_hid_ctrl_sail(int16_t speed1, int16_t speed2, int16_t dir)
{
	if(dir<=0)
	{
//		if(location_thread_switch == 1)
//		{
//			speed2 = motor_speed;
//		}
		speed1 = speed2+(dir*20);
		speed1 = (speed1<(-1000))?-1000:speed1;
	}
	if(dir>0)
	{
//		if(location_thread_switch == 1)
//		{
//			speed1 = motor_speed;
//		}
		
		speed2 = speed1-(dir*20);
		speed2 = (speed2<(-1000))?-1000:speed2;
	}
	sail_set_speed(speed1, speed2, dir);
	LOG_HMI("HMI: speed=%d, %d, dir=%d\r\n", speed1, speed2, dir);
	return 0;
}

extern int debug_switch;
static int on_hid_ctrl_throw(int16_t speed1, int16_t speed2)
{
	LOG_HMI("throw: speed1=%d, speed2==%d\r\n", speed1, speed2);
//	if(speed1 == 0)
//	{
//		debug_switch = 0;
//		LOG_HMI("debug switch off\r\n");
//	}
//	else
//	{
//		debug_switch = 1;
//		LOG_HMI("debug switch on\r\n");
//	}
	feed_set_speed(speed1, speed2);
	return 0;
}


//-------------------------------------------------------------------------------------------------
static void cmd_ctl_reboot(uint8_t token)
{
	if(token == 0xFF)
	{
		LOG_HMI("reboot...\r\n");
		board_reset();
	}
}

static void cmd_ctl_auto_navi(uint8_t ctl)
{
	if(ctl)
	{
		LOG_HMI("auto start...\r\n");
		svc_auto_start();
	}
	else
	{
		LOG_HMI("auto stop...\r\n");
		svc_auto_stop();
	}
}

static void cmd_set_navi_dest(uint8_t *data)
{
	int flag;
	struct track point;
	flag = data[0];
	byte_swap_order(&data[1], &point.longitude, 8);
	byte_swap_order(&data[9], &point.latitude, 8);
	track_write(flag, &point);
	//DUMP("data:", data, 16);
	LOG("write track:flag=%d, %.10lf, %.10lf\r\n", flag, point.longitude, point.latitude);
}

static void cmd_ctl_feed(uint8_t data)
{
	if(data == 0)
	{
		debug_switch = 0;
		LOG_HMI("debug switch off\r\n");
	}
	else
	{
		debug_switch = 1;
		LOG_HMI("debug switch on\r\n");
	}
	LOG("cmd_ctl_feed: not implemented!\r\n");
}

static int on_cfg_control(uint8_t type, uint8_t *data, int size)
{
	LOG("cfg ctl type=%02X.\r\n", type);
	switch(type)
	{
	case 0x00: //重启
		cmd_ctl_reboot(data[0]);
		return 0;
	case 0x01: //航行开关
		cmd_ctl_auto_navi(data[0]);
		return 0;
	case 0x02: //投料开关
		cmd_ctl_feed(data[0]);
		return 0;
	case 0x10: //罗盘校零
		svc_compass_cal_zero();
		return 0;
	case 0x11: //罗盘校准
		svc_compass_set_cali(data[0]);
		return 0;
	case 0x80: //设置导航坐标
		cmd_set_navi_dest(data);
		return 0;
	}
	return 1;
}

static int on_cfg_write(uint8_t type, uint8_t *data, int size)
{
	LOG("cfg set type=%02X.\r\n", type);
	switch(type)
	{
	case 0x80:
		byte_swap_order(&data[0], &appvar.pid_config[0][0], 4);
		byte_swap_order(&data[4], &appvar.pid_config[0][1], 4);
		byte_swap_order(&data[8], &appvar.pid_config[0][2], 4);
		LOG("PID1:%f,%f,%f\r\n", appvar.pid_config[0][0], appvar.pid_config[0][1], appvar.pid_config[0][2]);
		return 0;
	case 0x81:
		byte_swap_order(&data[0], &appvar.pid_config[1][0], 4);
		byte_swap_order(&data[4], &appvar.pid_config[1][1], 4);
		byte_swap_order(&data[8], &appvar.pid_config[1][2], 4);
		LOG("PID2:%f,%f,%f\r\n", appvar.pid_config[1][0], appvar.pid_config[1][1], appvar.pid_config[1][2]);
		return 0;
	case 0x82:
		byte_swap_order(data, &appvar.dst_course, 4);
		LOG("CRS:%d\r\n", appvar.dst_course);
		return 0;
	}
	return -1;
}

static int  on_cfg_read(uint8_t type, uint8_t *data)
{
	LOG("cfg get type=%02X.\r\n", type);
	switch(type)
	{
	case 0x80:
		byte_swap_order(&appvar.pid_config[0][0], &data[0], 4);
		byte_swap_order(&appvar.pid_config[0][1], &data[4], 4);
		byte_swap_order(&appvar.pid_config[0][2], &data[8], 4);
		return 12;
	case 0x81:
		byte_swap_order(&appvar.pid_config[1][0], &data[0], 4);
		byte_swap_order(&appvar.pid_config[1][1], &data[4], 4);
		byte_swap_order(&appvar.pid_config[1][2], &data[8], 4);
		return 12;
	case 0x82:
		byte_swap_order(&appvar.dst_course, data, 4);
		return 4;
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------
//工作线程
static void svc_hmi_thread(void *arg)
{
	LOG("[SVC:HMI]init\r\n");
	while(1)
	{
		//hmi_hid_send_batt(appvar.batt);
		//hmi_hid_send_sail(appvar.sail);
		//hmi_hid_send_feed(appvar.feed);
//		hmi_hid_send_gps(&appvar.gps);
		sleep(1000);
	}
}

void svc_hmi_init(void)
{
	struct hmi_handler handler;
	handler.on_cfg_read    = on_cfg_read;
	handler.on_cfg_write   = on_cfg_write;
	handler.on_cfg_control = on_cfg_control;
	handler.on_hid_connect = on_hid_connect;
	handler.on_hid_control_sail = on_hid_ctrl_sail;
	handler.on_hid_control_throw = on_hid_ctrl_throw;
	hmi_init();
	hmi_bind(&handler);
	thread_create(svc_hmi_thread, 0, 10240);
}
