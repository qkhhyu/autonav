/*
* 电子罗盘服务
***************************************************************************************************
    暂时不计算倾角
	xx = acc[0] * acc[0];
	yy = acc[1] * acc[1];
	zz = acc[2] * acc[2];
	roll =  atan2(acc[0], sqrt(yy + zz)) * 57.29577866666166f;//180.0f / 3.1415927f;
	pitch = atan2(acc[1], sqrt(xx + zz)) * 57.29577866666166f;
    hx = mx * cos(pitch) + my * sin(roll) * sin(pitch) - mz * sin(pitch) * cos(roll);
    hy = my * cos(roll) + mz * sin(roll);
    yaw = atan2(hy, hx) * 180.0f / 3.1415927f;
**************************************************************************************************
* 蒋晓岗<kerndev@foxmail.com>
* 2019.12.27
*/
#include <math.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "filter.h"

struct contex
{
	float x0;
	float y0;
	int cal_value;
	int min_value;
	int max_value;
};

static int m_cali_mode;
static int m_zero_offset;
static int m_heading_origin;
static struct contex m_contex[3];
static void(*m_handler)(int heading);

static float low_pass(struct contex *ctx, float x)
{
	float y;
	const float a1 = -0.726542f;
	const float ga = 0.136728f;
	x = (x + ctx->x0) / 2;
	y = x + ctx->x0 - ctx->y0 * a1;
	ctx->y0 = y;
	ctx->x0 = x;
	y = y * ga;
	return y;
}

//数据处理
static void proc_data(int16_t data[3])
{
	int i;
	int temp;
	struct contex *ctx;

	for(i=0; i<3; i++)
	{
		ctx = &m_contex[i];
		temp = low_pass(ctx, data[i]);
		data[i] = temp - ctx->cal_value;
		if(m_cali_mode)
		{
			if(ctx->min_value > temp)
			{
				ctx->min_value = temp;
				ctx->cal_value = (ctx->min_value + ctx->max_value) / 2;
			}
			if(ctx->max_value < temp)
			{
				ctx->max_value = temp;
				ctx->cal_value = (ctx->min_value + ctx->max_value) / 2;
			}
		}
	}
}

//计算朝向
static void calc_heading(int16_t mdm[3], int16_t pitch, int16_t roll)
{
	int heading;
	float hx;
	float hy;

	hx = mdm[0];
	hy = mdm[2];

	heading = atan2(hx, hy) * 180.0f / 3.1415927f;
	m_heading_origin = heading;
	heading = (heading + m_zero_offset + 360) % 359;
	appvar.heading = heading;
	//LOG("mdm=(%d, %d, %d), yaw=%d\r\n", mdm[0], mdm[1], mdm[2], heading);
	
	if(m_handler)
	{
		m_handler(heading);
	}
	
	static int div;
	if(++div > 20)
	{
		div = 0;
		LOG_HMI("[SVC:COMP],heading=%d,\r\n", heading);
	}
}


void svc_compass_input(int16_t mdm[3], int16_t acc[3])
{
	proc_data(mdm);
	calc_heading(mdm, 0, 0);
}

static void reset_contex(void)
{
	int i;
	struct contex *ctx;
	for(i=0; i<3; i++)
	{
		ctx = &m_contex[i];
		ctx->min_value = 32767;
		ctx->max_value = -32767;
		ctx->cal_value = 0;
	}
}

void svc_compass_set_cali(int ctl)
{
	if(ctl)
	{
		reset_contex();
		m_cali_mode = 1;
		LOG_HMI("[SVC:COMP]enter calibrate mode.\r\n");
	}
	else
	{
		m_cali_mode = 0;
		appcfg.compass_cal[0] = m_contex[0].cal_value;
		appcfg.compass_cal[1] = m_contex[1].cal_value;
		appcfg.compass_cal[2] = m_contex[2].cal_value;
		appcfg_write(&appcfg);
		LOG_HMI("[SVC:COMP]leave calibrate mode.\r\n");
	}
}

void svc_compass_cal_zero(void)
{
	m_zero_offset = -m_heading_origin;
	appcfg.compass_offset = m_zero_offset;
	appcfg_write(&appcfg);
	LOG_HMI("[SVC:COMP]zero offset=%d\r\n", m_zero_offset);
}

void svc_compass_bind(void(*handler)(int heading))
{
	m_handler = handler;
}

void svc_compass_init(void)
{
	m_handler = NULL;
	m_cali_mode = 0;
	m_zero_offset = appcfg.compass_offset;
	m_contex[0].cal_value = appcfg.compass_cal[0];
	m_contex[1].cal_value = appcfg.compass_cal[1];
	m_contex[2].cal_value = appcfg.compass_cal[2];
	LOG("[SVC:COMP]offset=%d\r\n", m_zero_offset);
	LOG("[SVC:COMP]cal.x =%d\r\n", m_contex[0].cal_value);
	LOG("[SVC:COMP]cal.y =%d\r\n", m_contex[1].cal_value);
	LOG("[SVC:COMP]cal.z =%d\r\n", m_contex[2].cal_value);
}
