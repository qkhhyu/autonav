/*
* 电源管理模块
* 电源管理模块挂接在485总线上
* 蒋晓岗<kerndev@foxmail.com>
* 2020.01.13
*/
#include <stdio.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "binconv.h"
#include "rs485.h"
#include "batt.h"

struct batt_contex
{
	uint8_t addr;
	uint8_t resp[16];
};

static struct batt_contex m_contex;

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

//查询电源状态
int batt_get_state(struct batt state[4])
{
	uint8_t data[8];
	data[0] = 0x02;
	data[1] = 0x00;
	if(!send_command(data, 2))
	{
		LOG("[BATT]get state failed: resp timeout!\r\n");
		return -1;
	}
	if(m_contex.resp[0] != data[0])
	{
		LOG("[BATT]get state failed: resp invalid!\r\n");
		return -2;
	}
	
	byte_swap_order(&m_contex.resp[1], &state[0].voltage, 2);
	byte_swap_order(&m_contex.resp[3], &state[0].temp,    1);
	byte_swap_order(&m_contex.resp[4], &state[0].state,   1);
	
	byte_swap_order(&m_contex.resp[5], &state[1].voltage, 2);
	byte_swap_order(&m_contex.resp[7], &state[1].temp,    1);
	byte_swap_order(&m_contex.resp[8], &state[1].state,   1);
	
	byte_swap_order(&m_contex.resp[9], &state[2].voltage, 2);
	byte_swap_order(&m_contex.resp[11], &state[2].temp,   1);
	byte_swap_order(&m_contex.resp[12], &state[2].state,  1);
	
	byte_swap_order(&m_contex.resp[13], &state[3].voltage, 2);
	byte_swap_order(&m_contex.resp[15], &state[3].temp,   1);
	byte_swap_order(&m_contex.resp[16], &state[3].state,  1);
	return 0;
}

void batt_init(void)
{
	m_contex.addr = 0x01;
}
