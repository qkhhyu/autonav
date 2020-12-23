/*
* 守护线程,负责看门狗
* 蒋晓岗<kerndev@foxmail.com>
* 2016.4.28
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"

//看门狗
static void wwdog_daemon(void* arg)
{
	int blink;
	thread_set_priority(thread_self(), THREAD_PRIORITY_IDLE);
	//wdog_init();
	for(blink=0;;blink=!blink)
	{
		sleep(500);
		//log_puts("");
		//wdog_feed();
	}
}

//CPU占用率
static void usage_daemon(void* arg)
{
	uint32_t tick;
	uint32_t idle;
	uint32_t used;
	uint32_t total;
	thread_set_priority(thread_self(), THREAD_PRIORITY_HIGH);
	while(1)
	{
		tick = kernel_time();
		idle = kernel_idle_time();
		sleep(10000);
		tick = kernel_time() - tick;
		idle = kernel_idle_time() - idle;
		heap_usage(&total, &used);
		LOG("[INFO]CPU:%2d%%, RAM:%dByte\r\n", 100*(tick-idle)/tick, used);
	}
}

void svc_daemon_init(void)
{
	thread_create(wwdog_daemon,0,10240);
	thread_create(usage_daemon,0,10240);
}
