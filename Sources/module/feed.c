/*
* 投料电机驱动模块
* 接在485总线上
* 蒋晓岗<kerndev@foxmail.com>
* 2020.01.13
*/
#include <stdio.h>
#include <string.h>
#include "kernel.h"
#include "app.h"
#include "bsp.h"
#include "log.h"
#include "crc.h"
#include "binconv.h"
#include "rs485.h"
#include "feed.h"

struct feed_contex
{
	uint8_t addr;
	uint8_t resp[8];
};

static struct feed_contex m_contex;

//接收数据处理
static void handler(uint8_t *data, int size)
{
	if(size < 8)
	{
		memcpy(m_contex.resp, data, size);
	}
}

//发送命令
static bool send_command(uint8_t *data, int size)
{
	bool ret;
	ret = rs485_data_xfer(m_contex.addr, data, size, handler);
	return ret;
}

//设置电机转速
int feed_set_speed(int16_t speed1, int16_t speed2)
{
	uint8_t data[8];
	data[0] = 0x01;
	byte_swap_order(&speed1, &data[1], 2);
	byte_swap_order(&speed2, &data[3], 2);
	if(!send_command(data, 5))
	{
		LOG("[FEED]set speed failed: resp timeout!\r\n");
		return -1;
	}
	if(m_contex.resp[0] != data[0])
	{
		LOG("[FEED]set speed failed: resp invalid!\r\n");
		return -2;
	}
	if(m_contex.resp[1] != 0x00)
	{
		LOG("[FEED]set speed failed: module error!\r\n");
		return -3;
	}
	return 0;
}

//查询电机状态
int feed_get_state(struct motor state[2])
{
	uint8_t data[8];
	data[0] = 0x02;
	data[1] = 0x00;
	if(!send_command(data, 2))
	{
		LOG("[FEED]get state failed: resp timeout!\r\n");
		return -1;
	}
	if(m_contex.resp[0] != data[0])
	{
		LOG("[FEED]get state failed: resp invalid!\r\n");
		return -2;
	}
	byte_swap_order(&m_contex.resp[1], &state[0].speed, 2);
	byte_swap_order(&m_contex.resp[3], &state[0].temp,  1);
	byte_swap_order(&m_contex.resp[4], &state[1].speed, 2);
	byte_swap_order(&m_contex.resp[6], &state[1].temp,  1);
	return 0;
}

//模块初始化
void feed_init(void)
{
	m_contex.addr = 0x03;
}
