/*
* GPS模块
* 暂时只解析RMC消息
* 格式：$xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode, navStatus*cs 
* 示例：$GNRMC,070748.000,V,,,,,,,231219,,E,N,V*6A
* 示例：$GNRMC,070749.000,A,2236.096759,N,11359.107361,E,0.271,82.60,231219,,E,A,V*4C
* 蒋晓岗<kerndev@foxmail.com>
* 2019.12.26
*/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "bsp.h"
#include "app.h"
#include "crc.h"
#include "binconv.h"
#include "gps.h"

#define UART_PORT	4
#define UART_BAUD   115200

void (*m_handler)(struct gps *gps);

//拆分GPS数据
static char *gps_split_data(char *data, char *out)
{
	while((*data != ',') && (*data != '\r'))
	{
		*out++ = *data++;
	}
	*out = 0;
	return data;
}

//处理帧数据
static void gps_proc_data(char *data)
{
	int num;
	char dump[16];
	struct gps gps;
	if(strncmp(data+3, "RMC", 3) == 0)
	{
		//data = "$GNRMC,023140.000,A,2235.548092,N,11359.255034,E,0.473,8.02,060120,,E,A,V*7A\r\n";
		//LOG(data);
		memset(&gps, 0, sizeof(gps));
		data = gps_split_data(data+7, dump); //时间hhmmss.sss
		data = gps_split_data(data+1, dump); //定位标识A/V
		gps.locked = (dump[0] == 'A');
		
		data = gps_split_data(data+1, dump); //纬度ddmm.mmmmmm
		sscanf(dump, "%02d%lf", &num, &gps.latitude);
		gps.latitude = num + gps.latitude / 60;
		data = gps_split_data(data+1, dump); //南北纬N/S
		gps.latitude = (dump[0] == 'N') ? (gps.latitude) : (-gps.latitude);
		
		data = gps_split_data(data+1, dump); //经度dddmm.mmmmmm
		sscanf(dump, "%03d%lf", &num, &gps.longitude);
		gps.longitude = num + gps.longitude / 60;
		data = gps_split_data(data+1, dump); //东西经E/W
		gps.longitude = (dump[0] == 'E') ? (gps.longitude) : (-gps.longitude);
		
		data = gps_split_data(data+1, dump); //速率(a.bbb)
		sscanf(dump, "%f", &gps.speed);
		
		data = gps_split_data(data+1, dump); //航向(aaa.bbb)
		sscanf(dump, "%f", &gps.course);
		
		if(m_handler)
		{
			m_handler(&gps);
		}
	}
}

//接收帧数据
static void gps_recv_data(char *stream, int size)
{
	static char frame[260];
	static int  frame_size;
	int i;
	for(i=0; i<size; i++)
	{
		frame[frame_size++] = stream[i];
		if(frame_size == 1)
		{
			if(stream[i] != '$')
			{
				frame_size = 0;
			}
			continue;
		}
		if(frame_size == 256)
		{
			frame_size = 0;
			continue;
		}
		if(stream[i] == '\n')
		{
			frame[frame_size] = 0;
			frame_size = 0;
			gps_proc_data(frame);
		}
	}
}

static void gps_thread(void *arg)
{
	char buf[260];
	int ret;
	while(1)
	{
		ret = uart_read(UART_PORT, buf, 256);
		if(ret > 0)
		{
			gps_recv_data(buf, ret);
		}
		else
		{
			sleep(50);
		}
	}
}

void gps_bind(void(*handler)(struct gps *gps))
{
	m_handler = handler;
}

void gps_init(void)
{
	m_handler = NULL;
	gpio_open(PI, 0, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(PI, 0, 0);
	uart_init(UART_PORT, heap_alloc(1024), 1024);
	uart_open(UART_PORT, UART_BAUD, 0);
	thread_create(gps_thread, 0, 10240);
}
