/*
* 485总线服务
* 定时查询各模块状态
* 蒋晓岗<kerndev@foxmail.com>
* 2020.01.13
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "rs485.h"
#include "batt.h"
#include "sail.h"
#include "feed.h"

static void svc_485_thread(void *arg)
{
	LOG("[SVC:485]init\r\n");
	while(1)
	{
		//batt_get_state(appvar.batt);
		sail_get_state(appvar.sail);
		//feed_get_state(appvar.feed);
		sleep(1000);
	}
}

void svc_485_init(void)
{
	rs485_init();
	batt_init();
	sail_init();
	feed_init();
	thread_create(svc_485_thread, 0, 10240);
}
