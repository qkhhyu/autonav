/*
* ²â¾à·þÎñ
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2019.12.27
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "dist.h"

static void(*m_handler)(struct dist *dist);

static void svc_dist_thread(void *arg)
{
	struct dist dist;
	LOG("[SVC:DIST]start.\r\n");
	while(1)
	{
		sleep(100);
		dist_read(0x01, &dist.front);
		dist_read(0x02, &dist.back);
		dist_read(0x03, &dist.right1);
		dist_read(0x04, &dist.right2);
		//LOG("[SVC:DIST]f=%d, b=%d, r1=%d, r2=%d\r\n", dist.front, dist.back, dist.right1, dist.right2);
		memcpy(&appvar.dist, &dist, sizeof(dist));
		//hmi_hid_send_dist(&appvar.dist);
		if(m_handler)
		{
			m_handler(&dist);
		}
	}
}

void svc_dist_bind(void(*handler)(struct dist *dist))
{
	m_handler = handler;
}

void svc_dist_init(void)
{
	dist_init();
	thread_create(svc_dist_thread, 0, 10240);
}
