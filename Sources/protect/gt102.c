/*
* ZL1GT02安全芯片驱动,采用SHA-256算法
* 注意：芯片页面数据不可改写,出厂数据为0x00或0xAA
* 蒋晓岗<kerndev@foxmail.com>
*/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "gt102_def.h"
#include "gt102.h"

#define I2C_PORT    0

static int gt102_write(uint8_t addr, uint8_t *data, int len)
{
	return i2c_xfer_tx(I2C_PORT, I2C_ADDR, addr, data, len);
}

static int gt102_read(uint8_t addr, uint8_t *data, int len)
{
	return i2c_xfer_rx(I2C_PORT, I2C_ADDR, addr, data, len);
}

static int gt102_execute_status(void)
{
	int i;
	uint8_t buff[4];
	for(i=0;i<10000;i++)
	{
		gt102_read(ADDR_ES,buff,1);
		buff[0] &= 0x11;
		if(buff[0] != 0x00)
		{
			return (buff[0] == 0x01);
		}
	}
	return 0;
}

//设置用户ID
int gt102_set_uid(uint8_t usid[8])
{
	int ret;
	uint8_t buff[4];
	buff[0] = CMD_CMDCLR;
	gt102_write(ADDR_CMD, buff, 1);
	gt102_write(ADDR_MEMBUF, usid, 8);
	buff[0] = CMD_INITUSID;
	gt102_write(ADDR_CMD, buff, 1);
	ret = gt102_execute_status();
	if(!ret)
	{
		return 0;
	}
	buff[0] = 0x5A;
	gt102_write(ADDR_UIDPRO, buff, 1);
	return 1;
}

//设置密钥
int gt102_set_key(uint8_t key[8])
{
	int ret;
	uint8_t buff[4];
	buff[0] = CMD_CMDCLR;
	gt102_write(ADDR_CMD, buff, 1);
	gt102_write(ADDR_MEMBUF, key, 8);
	buff[0] = CMD_INITKEY;
	gt102_write(ADDR_CMD, buff, 1);
	ret = gt102_execute_status();
	if(!ret)
	{
		return 0;
	}
	buff[0] = 0x5A;
	gt102_write(ADDR_KEYPRO, buff, 1);
	return 1;
}

//初始化页面数据
//page:0-3要设置的页面
//data:32字节页面数据
int gt102_set_page(uint8_t page, uint8_t data[32])
{
	uint8_t buff[4];
	buff[0] = CMD_CMDCLR;
	gt102_write(ADDR_CMD, buff, 1);
	gt102_write(ADDR_MEMBUF, data, 32);
	buff[0] = page;
	gt102_write(ADDR_TA_DST, buff, 1);
	buff[0] = CMD_INITPAGE;
	gt102_write(ADDR_CMD, buff, 1);
	return gt102_execute_status();
}

//开始认证
//page:0-3参与计算的页面
//input: 参与计算的8字节随机数
//output: 芯片计算的SHA256值
int gt102_auth_device(uint8_t page, uint8_t input[8], uint8_t output[32])
{
	int i;
	int ret;
	uint8_t buff[4];
	buff[0] = CMD_CMDCLR;
	gt102_write(ADDR_CMD, buff, 1);
	gt102_write(ADDR_MEMBUF, input, 8);
	buff[0] = page;
	gt102_write(ADDR_TA_SRC, buff, 1);
	buff[0] = CMD_AUTHDEV;
	gt102_write(ADDR_CMD, buff, 1);
	ret = gt102_execute_status();
	for(i=0;i<32;i+=4)
	{
		gt102_read(ADDR_MEMBUF+i, output+28-i, 4);
		
	}
	//gt102_read(ADDR_MEMBUF, output, 32);
	
	return ret;
}

int gt102_version(uint32_t* output)
{
	int ret;
	ret = gt102_read(ADDR_VERSION0, (uint8_t*)output,4);
	return ret;
}

void gt102_init(void)
{

}
