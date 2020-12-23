/*
* HMI模块
* 负责人机交互
* 蒋晓岗<kerndev@foxmail.com>
* 2019.12.26
*/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "kernel.h"
#include "bsp.h"
#include "log.h"
#include "crc.h"
#include "binconv.h"
#include "hmi.h"

#define UART_PORT	3
#define UART_BAUD   115200
#define GPIO_CTRL   PH, 12

struct contex
{
	mutex_t mutex;
	struct hmi_handler handler;
};

static struct contex m_contex;

//计算检验和
static uint8_t hmi_calc_checksum(uint8_t *data, int size)
{
	return crc8_calc(0, data, size);
}

//发送命令
static void hmi_send_data(uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t frame[260];
	frame[0] = 0xA5;
	frame[1] = size + 2;
	frame[2] = addr;
	memcpy(&frame[3], data, size);
	frame[3 + size] = hmi_calc_checksum(frame, size + 3);
	mutex_lock(m_contex.mutex);
	uart_write(UART_PORT, frame, 4 + size);
	mutex_unlock(m_contex.mutex);
	//DUMP("[HMI]send:", frame, 4 + size);
}

//发送调试日志
static void hmi_dbg_send(void *data, int size)
{
	hmi_send_data(0x00, data, size);
}

//发送遥控数据
static void hmi_hid_send(void *data, int size)
{
	hmi_send_data(0x01, data, size);
}

//发送参数数据
static void hmi_cfg_send(void *data, int size)
{
	hmi_send_data(0x02, data, size);
}

//-------------------------------------------------------------------------------------------------
//调试通道
void hmi_dbg_dump(char *str, void *mem, int size)
{
	int  i;
	int  len;
	char text[260];
	len = sprintf(text, str);
	for(i=0; i<size; i++)
	{
		len += sprintf(&text[len], "%02X", ((char *)mem)[i]);
	}
	len += sprintf(&text[len], "\r\n");
	hmi_dbg_send(text, len);
	LOG("%s", text);
}

void hmi_dbg_puts(char *fmt, ...)
{
	int len;
	char text[260];
	va_list va;
	va_start(va, fmt);
	len = vsnprintf(text, 260, fmt, va);
	va_end(va);
	hmi_dbg_send(text, len);
	LOG("%s", text);
}


//-------------------------------------------------------------------------------------------------
//遥控器:处理连接通知
static void hid_proc_connect(uint8_t cmd, uint8_t *data, uint8_t size)
{
	uint8_t resp[2];
	if(m_contex.handler.on_hid_connect)
	{
		m_contex.handler.on_hid_connect(data[0]);
	}
	resp[0] = cmd;
	resp[1] = data[0];
	hmi_hid_send(resp, 2);
}


//遥控器:处理航行控制
static void hid_proc_ctrl_sail(uint8_t cmd, uint8_t *data, uint8_t size)
{
	int ret;
	int16_t val[3];
	uint8_t resp[2];
	byte_swap_order(&data[0], &val[0], 2);
	byte_swap_order(&data[2], &val[1], 2);
	byte_swap_order(&data[4], &val[2], 2);
	ret = -1;
	if(m_contex.handler.on_hid_control_sail)
	{
		ret = m_contex.handler.on_hid_control_sail(val[0], val[1], val[2]);
	}
	resp[0] = cmd;
	resp[1] = ret;
	hmi_hid_send(resp, 2);
}

//遥控器:处理投料控制
static void hid_proc_ctrl_throw(uint8_t cmd, uint8_t *data, uint8_t size)
{
	int ret;
	int16_t val[2];
	uint8_t resp[2];
	byte_swap_order(&data[0], &val[0], 2);
	byte_swap_order(&data[2], &val[1], 2);
	ret = -1;
	if(m_contex.handler.on_hid_control_throw)
	{
		ret = m_contex.handler.on_hid_control_throw(val[0], val[1]);
	}
	resp[0] = cmd;
	resp[1] = ret;
	hmi_hid_send(resp, 2);
}

//遥控器:处理命令
static void hid_proc_command(uint8_t cmd, uint8_t *data, uint8_t size)
{
	if(cmd == 0x10)
	{
		hid_proc_connect(cmd, data, size);
		return;
	}
	if(cmd == 0x11)
	{
		hid_proc_ctrl_sail(cmd, data, size);
		return;
	}
	if(cmd == 0x12)
	{
		hid_proc_ctrl_throw(cmd, data, size);
		return;
	}
	LOG("hid unknow cmd:%02X\r\n", cmd);
}

//发送GPS状态
void hmi_hid_send_gps(struct gps *gps)
{
	uint8_t data[32];
	data[0] = 0x01;
	data[1] = gps->locked;
	byte_swap_order(&gps->latitude, &data[2], 8);
	byte_swap_order(&gps->longitude, &data[10], 8);
	byte_swap_order(&gps->speed, &data[18], 4);
	byte_swap_order(&gps->course, &data[22], 4);
	byte_swap_order(&gps->course, &data[26], 4);
//	hmi_hid_send(data, 30);
}

//发送测距信息
void hmi_hid_send_dist(struct dist *dist)
{
	uint8_t data[9];
	data[0] = 0x02;
	byte_swap_order(&dist->front, &data[1], 2);
	byte_swap_order(&dist->back,  &data[3], 2);
	byte_swap_order(&dist->right1, &data[5], 2);
	byte_swap_order(&dist->right2, &data[7], 2);
	hmi_hid_send(data, 9);
}

//发送电机状态
void hmi_hid_send_sail(struct motor sail[3])
{
	uint8_t data[12];
	data[0] = 0x03;
	byte_swap_order(&sail[0].speed, &data[1], 2);
	byte_swap_order(&sail[0].temp,  &data[3], 1);
	byte_swap_order(&sail[1].speed, &data[4], 2);
	byte_swap_order(&sail[1].temp,  &data[6], 1);
	byte_swap_order(&sail[2].speed, &data[7], 2);
	byte_swap_order(&sail[2].temp,  &data[9], 1);
	hmi_hid_send(data, 10);
}

//发送投料状态
void hmi_hid_send_feed(struct motor feed[2])
{
	uint8_t data[8];
	data[0] = 0x04;
	byte_swap_order(&feed[0].speed, &data[1], 2);
	byte_swap_order(&feed[0].temp,  &data[3], 1);
	byte_swap_order(&feed[1].speed, &data[4], 2);
	byte_swap_order(&feed[1].temp,  &data[6], 1);
	hmi_hid_send(data, 7);
}

//发送电源状态
void hmi_hid_send_batt(struct batt batt[3])
{
	uint8_t data[13];
	
	data[0] = 0x05;
	byte_swap_order(&batt[0].voltage, &data[1], 2);
	byte_swap_order(&batt[0].temp,    &data[3], 1);
	byte_swap_order(&batt[0].state,   &data[4], 1);
	
	byte_swap_order(&batt[1].voltage, &data[5], 2);
	byte_swap_order(&batt[1].temp,    &data[7], 1);
	byte_swap_order(&batt[1].state,   &data[8], 1);
	
	byte_swap_order(&batt[2].voltage, &data[9], 2);
	byte_swap_order(&batt[2].temp,    &data[11], 1);
	byte_swap_order(&batt[2].state,   &data[12], 1);

	hmi_hid_send(data, 13);
}


//-------------------------------------------------------------------------------------------------
//参数配置：处理命令
static void cfg_proc_command(uint8_t cmd, uint8_t *data, uint8_t size)
{
	int ret;
	uint8_t resp[256];
	if(cmd == 0x20)
	{
		LOG("cfg cmd\r\n");
		ret = -1;
		if(m_contex.handler.on_cfg_write)
		{
			ret = m_contex.handler.on_cfg_control(data[0], &data[1], size - 1);
		}
		resp[0] = cmd;
		resp[1] = ret;
		hmi_send_data(0x02, resp, 2);
		return;
	}
	if(cmd == 0x21)
	{
		LOG("cfg set\r\n");
		ret = -1;
		if(m_contex.handler.on_cfg_write)
		{
			ret = m_contex.handler.on_cfg_write(data[0], &data[1], size - 1);
		}
		resp[0] = cmd;
		resp[1] = ret;
		hmi_send_data(0x02, resp, 2);
		return;
	}
	if(cmd == 0x22)
	{
		LOG("cfg get\r\n");
		ret = 0;
		resp[0] = cmd;
		resp[1] = data[0];
		if(m_contex.handler.on_cfg_read)
		{
			ret = m_contex.handler.on_cfg_read(data[0], &resp[2]);
		}
		hmi_send_data(0x02, resp, ret + 2);
		return;
	}
	LOG("cfg unknow cmd:%02X\r\n", cmd);
}


//处理帧数据
static void hmi_proc_data(uint8_t addr, uint8_t *data, uint8_t size)
{
	if(addr == 0x00)
	{
		LOG("HMI:recv dbg data!\r\n");
		return;
	}
	if(addr == 0x01)
	{
		LOG("HMI:recv hid data!\r\n");
		hid_proc_command(data[0], &data[1], size - 1);
		return;
	}
	if(addr == 0x02)
	{
		LOG("HMI:recv cfg data!\r\n");
		cfg_proc_command(data[0], &data[1], size - 1);
		return;
	}
	LOG("HMI:recv unknow data: addr=0x%02X\r\n", addr);
}

//接收帧数据
static void hmi_recv_data(uint8_t *stream, int size)
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
			frame_size = 0;
			hmi_proc_data(frame[2], &frame[3], frame[1] - 2);
		}
	}
}

static void hmi_recv_thread(void *arg)
{
	uint8_t buf[260];
	int ret;
	while(1)
	{
		ret = uart_read(UART_PORT, buf, 256);
		if(ret > 0)
		{
			hmi_recv_data(buf, ret);
		}
		else
		{
			sleep(20);
		}
	}
}

void hmi_bind(struct hmi_handler *handler)
{
	if(handler)
	{
		memcpy(&m_contex.handler, handler, sizeof(struct hmi_handler));
	}
}

void hmi_init(void)
{
	memset(&m_contex, 0, sizeof(m_contex));
	m_contex.mutex = mutex_create();
	gpio_open(GPIO_CTRL, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(GPIO_CTRL, 0);
	uart_init(UART_PORT, heap_alloc(1024), 1024);
	uart_open(UART_PORT, UART_BAUD, 0);
	thread_create(hmi_recv_thread, 0, 10240);
}
