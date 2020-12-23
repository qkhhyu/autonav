/*
* 基于加速度的撞击检测
* 蒋晓岗<kerndev@foxmail.com>
* 2020.09.09
*/
#include <math.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "svc.h"

#define HIT_VALUE 4000
#define HIT_SKIP  100

struct contex
{
	float x0;
	float y0;
};

struct contex m_contex[3];
static int    m_skip;
static void (*m_handler)(uint8_t evt);

static float high_pass(struct contex *ctx, float x)
{
	float y;
	y  = x - ctx->x0 + ctx->y0 * 0.726542f;
	ctx->y0 = y;
	ctx->x0 = x;
	return y;
}

static void post_event(uint8_t evt)
{
	if(evt)
	{
		LOG("[SVC:HIT]post evt=%02X\r\n", evt);
		if(m_handler)
		{
			m_handler(evt);
		}
	}
}

void svc_hittest_input(int16_t data[3])
{
	int i;
	uint8_t evt;
	struct contex *ctx;
	
	evt = 0;
	for(i=0; i<3; i++)
	{
		ctx = &m_contex[i];
		data[i] = high_pass(ctx, data[i]);
	}
	if(m_skip)
	{
		m_skip--;
		return;
	}
	for(i=0; i<3; i++)
	{
		if(data[i] > HIT_VALUE)
		{
			LOG("%d hit +\r\n", i);
			m_skip = HIT_SKIP;
			evt |= 0x01 << i;
		}
		if(data[i] < -HIT_VALUE)
		{
			LOG("%d hit -\r\n", i);
			m_skip = HIT_SKIP;
			evt |= 0x10 << i;
		}
	}
	post_event(evt);
	//LOG("%d,%d,%d\r\n",data[0], data[1], data[2]);
}

void svc_hittest_bind(void(*handler)(uint8_t evt))
{
	m_handler = handler;
}

void svc_hittest_init(void)
{
	m_handler = NULL;
	m_skip = 100;
	memset(&m_contex, 0, sizeof(m_contex));
}
