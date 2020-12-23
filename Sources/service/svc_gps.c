/*
* GPS定位服务
* 蒋晓岗<kerndev@foxmail.com>
* 2019.12.27
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "gps.h"

static void(*m_handler)(struct gps *gps);

static void gps_handler(struct gps *gps)
{
	LOG("[SVC:GPS]%s,%+.9lf,%+.9lf,speed=%f, course=%f\r\n", gps->locked ? "OK":"NA",  gps->latitude, gps->longitude, gps->speed, gps->course);
	memcpy(&appvar.gps, gps, sizeof(struct gps));
	if(m_handler)
	{
		m_handler(gps);
	}
}

void svc_gps_bind(void(*handler)(struct gps *gps))
{
	m_handler = handler;
}

void svc_gps_start(void)
{
	
}

void svc_gps_stop(void)
{
	
}

void svc_gps_init(void)
{
	m_handler = NULL;
	gps_init();
	gps_bind(gps_handler);
}
