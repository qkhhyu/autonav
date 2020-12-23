/*
* 自动巡航服务
* 蒋晓岗<kerndev@foxmail.com>
* 2020.08.03
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "sail.h"
#include "feed.h"

#include "auto_location.h"
#include "auto_boat_cruise.h"
#include "auto_boat.h"
#include "auto_dist.h"

//工作线程
static void svc_auto_thread(void *arg)
{
	LOG("[SVC:AUTO]init\r\n");
	auto_boat_init();
	auto_cruise_init();
	auto_location_init();
	auto_dist_init();
	while(1)
	{
		//LOG("[SVC:AUTO]run\r\n");
		sleep(1000);
	}
}

//启动自动导航
void svc_auto_start(void)
{
	LOG("[SVC:AUTO]start\r\n");
	location_thread_switch = 1;
	auto_location_resume();
	
}

//关闭自动导航
void svc_auto_stop(void)
{
	LOG("[SVC:AUTO]stop\r\n");
	location_thread_switch = 0;
}


void svc_auto_init(void)
{
	thread_create(svc_auto_thread, 0, 10240);
}
