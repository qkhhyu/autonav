/*
* 485总线通信模块
* 485总线挂接一个电源管理模块，一个动力推进电机驱动，一个投料装置
* 蒋晓岗<kerndev@foxmail.com>
* 2020.01.13
*/

#include <stdio.h>
#include <string.h>
#include "kernel.h"
#include "bsp.h"
#include "log.h"
#include "crc.h"
#include "binconv.h"
#include "rs485.h"

#define UART_PORT	7
#define UART_BAUD   115200
#define GPIO_CTRL   PF, 8

struct contex
{
	event_t event;
	mutex_t mutex;
	uint8_t addr;
	rs485_data_handler_t handler;
};

static struct contex m_contex;

//计算检验和
static uint8_t rs485_calc_checksum(uint8_t *data, int size)
{
	return crc8_calc(0, data, size);
}

//发送数据
static void rs485_send_data(uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t frame[260];
	
	frame[0] = 0xA5;
	frame[1] = size + 2;
	frame[2] = addr;
	memcpy(&frame[3], data, size);
	frame[3 + size] = rs485_calc_checksum(frame, size + 3);
	
	gpio_write(GPIO_CTRL, 1);
	uart_write(UART_PORT, frame, 4 + size);
	gpio_write(GPIO_CTRL, 0);
	//DUMP("[485]send:", frame, 4 + size);
}

//总线数据交换
bool rs485_data_xfer(uint8_t addr, void *data, int size, rs485_data_handler_t handler)
{
	bool ret;
	mutex_lock(m_contex.mutex);
	event_reset(m_contex.event);
	m_contex.addr = addr;
	m_contex.handler = handler;
	
	rs485_send_data(addr, data, size);
	
	ret = event_timed_wait(m_contex.event, 1000);
	m_contex.addr = 0;
	m_contex.handler = NULL;
	mutex_unlock(m_contex.mutex);
	return ret;
}

//处理帧数据
static void rs485_proc_frame(uint8_t addr, uint8_t *data, uint8_t size)
{
	if(m_contex.addr != addr)
	{
		LOG("[485]addr mismatch: send=%02X, recv=%02X!\r\n", m_contex.addr, addr);
		return;
	}
	if(m_contex.handler != NULL)
	{
		m_contex.handler(data, size);
	}
	event_post(m_contex.event);
}

//接收帧数据
static void rs485_recv_frame(uint8_t *stream, int size)
{
	static uint8_t frame[260];
	static uint8_t frame_size;
	int i;
	for(i=0; i<size; i++)
	{
		frame[frame_size++] = stream[i];
		if(frame_size == 1)
		{
			if(stream[i] != 0xA5)
			{
				frame_size = 0;
			}
			continue;
		}
		if(frame_size == 2)
		{
			if(stream[i] < 2)
			{
				frame_size = 0;
			}
			continue;
		}
		if(frame_size == frame[1] + 2)
		{
			DUMP("[485]recv:", frame, frame_size);
			frame_size = 0;
			rs485_proc_frame(frame[2], &frame[3], frame[1] - 2);
		}
	}
}

//接收线程
static void rs485_recv_thread(void *arg)
{
	uint8_t buf[260];
	int ret;
	while(1)
	{
		ret = uart_read(UART_PORT, buf, 256);
		if(ret > 0)
		{
			rs485_recv_frame(buf, ret);
		}
		else
		{
			sleep(50);
		}
	}
}

//初始化
void rs485_init(void)
{
	memset(&m_contex, 0, sizeof(m_contex));
	m_contex.event = event_create();
	m_contex.mutex = mutex_create();
	gpio_open(GPIO_CTRL, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(GPIO_CTRL, 0);
	uart_init(UART_PORT, heap_alloc(1024), 1024);
	uart_open(UART_PORT, UART_BAUD, 0);
	thread_create(rs485_recv_thread, 0, 10240);
}
