/*
* 航行电机模块
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
#include "sail.h"

struct sail_contex
{
	uint8_t addr;
	uint8_t resp[16];
};

static struct sail_contex m_contex;

//接收数据处理
static void handler(uint8_t *data, int size)
{
	if(size < 16)
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
int sail_set_speed(int16_t speed1, int16_t speed2, int16_t angle)
{
	uint8_t data[8];
	data[0] = 0x01;
	byte_swap_order(&speed1, &data[1], 2);
	byte_swap_order(&speed2, &data[3], 2);
	byte_swap_order(&angle,  &data[5], 2);
	if(!send_command(data, 7))
	{
		LOG("[SAIL]set speed failed: resp timeout!\r\n");
		return -1;
	}
	if(m_contex.resp[0] != data[0])
	{
		LOG("[SAIL]set speed failed: resp invalid!\r\n");
		return -2;
	}
	if(m_contex.resp[1] != 0x00)
	{
		LOG("[SAIL]set speed failed: module error!\r\n");
		return -3;
	}
	return 0;
}

//查询电机状态
int sail_get_state(struct motor state[3])
{
	uint8_t data[8];
	data[0] = 0x02;
	data[1] = 0x00;
	if(!send_command(data, 2))
	{
		LOG("[SAIL]get state failed: resp timeout!\r\n");
		return -1;
	}
	if(m_contex.resp[0] != data[0])
	{
		LOG("[SAIL]get state failed: resp invalid!\r\n");
		return -2;
	}
	byte_swap_order(&m_contex.resp[1], &state[0].speed, 2);
	byte_swap_order(&m_contex.resp[3], &state[0].temp,  1);
	byte_swap_order(&m_contex.resp[4], &state[1].speed, 2);
	byte_swap_order(&m_contex.resp[6], &state[1].temp,  1);
	byte_swap_order(&m_contex.resp[7], &state[2].speed, 2);
	byte_swap_order(&m_contex.resp[9], &state[2].temp,  1);
	LOG("[SAIL]state: m0=%d, m1=%d, dir=%d\r\n", state[0].speed, state[1].speed, state[2].speed);
	return 0;
}

//模块初始化
void sail_init(void)
{
	m_contex.addr = 0x02;
}
