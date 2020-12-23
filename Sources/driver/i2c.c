/******************************************************************************
* I2C驱动(GPIO模拟)
* 蒋晓岗<kerndev@foxmail.com>
* 2020.09.22
******************************************************************************/
#include <stdint.h>
#include "delay.h"
#include "gpio.h"
#include "i2c.h"

#define	SCK_SET(ctx,x)	gpio_write((ctx)->sck_port,(ctx)->sck_pin,(x))
#define	SCK_GET(ctx)	gpio_read((ctx)->sck_port,(ctx)->sck_pin)
#define	SDA_SET(ctx,x)	gpio_write((ctx)->sda_port,(ctx)->sda_pin,(x))
#define	SDA_GET(ctx)	gpio_read((ctx)->sda_port,(ctx)->sda_pin)
#define DELAY(x)        delay_us(x)

struct i2c_contex
{
	int sck_port;
	int sck_pin;
	int sda_port;
	int sda_pin;
};

#define GET_HW_CTX(id)  (&m_i2c_ctx[id])
static struct i2c_contex m_i2c_ctx[4]=
{
	//SCK   SDA
    {PA,12, PA,11},
    {PA, 8, PC, 9},
	{PC,10, PC,12},
};

//初始化GPIO
static void i2c_gpio_init(int id)
{
	struct i2c_contex *ctx;
	ctx = GET_HW_CTX(id);
	gpio_open(ctx->sck_port, ctx->sck_pin, GPIO_MODE_OUT, GPIO_OUT_OD);
	gpio_open(ctx->sda_port, ctx->sda_pin, GPIO_MODE_OUT, GPIO_OUT_OD);
	SDA_SET(ctx, 1);
	SCK_SET(ctx, 1);
}

//产生起始条件
static void i2c_start(struct i2c_contex *ctx)
{
	SDA_SET(ctx, 1);
	SCK_SET(ctx, 1);
	DELAY(2);
	SDA_SET(ctx, 0);
	DELAY(2);
	SCK_SET(ctx, 0);
}

//产生停止条件
static void i2c_stop(struct i2c_contex *ctx)
{
	SCK_SET(ctx, 0);
	SDA_SET(ctx, 0);
	DELAY(2);
	SCK_SET(ctx, 1);
	DELAY(2);
	SDA_SET(ctx, 1);
}

//等待应答
static uint8_t i2c_wait_ack(struct i2c_contex *ctx)
{
	uint8_t ack;
	SDA_SET(ctx, 1);
	SCK_SET(ctx, 1);
	DELAY(2);
	ack = !SDA_GET(ctx);
	SCK_SET(ctx, 0);
	return ack;
}

//发送应答
static void i2c_send_ack(struct i2c_contex *ctx, uint8_t ack)
{
	SCK_SET(ctx, 0);
	SDA_SET(ctx, !ack);
	DELAY(2);
	SCK_SET(ctx, 1);
	DELAY(2);
	SCK_SET(ctx, 0);
}

//发送字节
static void i2c_send_byte(struct i2c_contex *ctx, uint8_t data)
{
	int i;
	for(i=0; i<8; i++)
	{
		SDA_SET(ctx, (data & 0x80) >> 7);
		data <<= 1;
		DELAY(2);
		SCK_SET(ctx, 1);
		DELAY(2);
		SCK_SET(ctx, 0);
	}
}

//接收字节
static uint8_t i2c_recv_byte(struct i2c_contex *ctx)
{
	int i;
	uint8_t data;
	data = 0;
	SDA_SET(ctx, 1);
	for(i=0; i<8; i++)
	{
		SCK_SET(ctx, 0);
		DELAY(2);
		SCK_SET(ctx, 1);
		data <<= 1;
        data |= SDA_GET(ctx);
		DELAY(2);
	}
	return data;
}

//写从机数据
int i2c_xfer_tx(int id, uint8_t addr, uint8_t reg, uint8_t *buf, int size)
{
    int i;
    int ret;
    struct i2c_contex *ctx;
    ctx = GET_HW_CTX(id);
	
	i2c_start(ctx);
	i2c_send_byte(ctx, addr&0xfe);
	ret = i2c_wait_ack(ctx);
	if(!ret)
	{
		i2c_stop(ctx);
		return -1;
	}
	i2c_send_byte(ctx, reg);
	ret = i2c_wait_ack(ctx);
	if(!ret)
	{
		i2c_stop(ctx);
		return -2;
	}
    for(i=0; i<size; i++)
    {
        i2c_send_byte(ctx, buf[i]);
		ret = i2c_wait_ack(ctx);
		if(!ret)
		{
			i2c_stop(ctx);
			return -3;
		}
    }
	i2c_stop(ctx);
	return 0;
}

//读从机数据
int i2c_xfer_rx(int id, uint8_t addr, uint8_t reg, uint8_t *buf, int size)
{
    int i;
    int ret;
    struct i2c_contex *ctx;
    ctx = GET_HW_CTX(id);
	i2c_start(ctx);
	i2c_send_byte(ctx, addr&0xfe);
	ret = i2c_wait_ack(ctx);
	if(!ret)
	{
		i2c_stop(ctx);
		return -1;
	}
	i2c_send_byte(ctx, reg);
	ret = i2c_wait_ack(ctx);
	if(!ret)
	{
		i2c_stop(ctx);
		return -2;
	}
	i2c_start(ctx);
	i2c_send_byte(ctx, addr|0x01);
	ret = i2c_wait_ack(ctx);
	if(!ret)
	{
		i2c_stop(ctx);
		return -3;
	}
    for(i=0; i<size; i++)
    {
        buf[i] = i2c_recv_byte(ctx);
		i2c_send_ack(ctx, i!=(size-1));
    }
	i2c_stop(ctx);
	return 0;
}

//初始化
void i2c_init(void)
{
    i2c_gpio_init(0);
    i2c_gpio_init(1);
	i2c_gpio_init(2);
}
