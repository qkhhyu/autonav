/*
* STM32F4xx GPIOÇý¶¯
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2016.6.12
*/
#include "stm32f4xx.h"
#include "gpio.h"

#define GET_HW_CTX(port)	((GPIO_TypeDef *)(AHB1PERIPH_BASE + ((port)<<10)))

void gpio_open(int port, int pin, int mode, int type)
{
	GPIO_InitTypeDef init;
	
    init.GPIO_Mode  = (GPIOMode_TypeDef)mode;
	init.GPIO_PuPd  = (mode == GPIO_MODE_IN) ? (GPIOPuPd_TypeDef)type : GPIO_PuPd_UP;
	init.GPIO_OType = (mode == GPIO_MODE_OUT) ? (GPIOOType_TypeDef)type : GPIO_OType_PP;
	init.GPIO_Speed = GPIO_Speed_100MHz;
	init.GPIO_Pin   = 1<<pin;
	GPIO_Init(GET_HW_CTX(port), &init);
	if(mode == GPIO_MODE_AF)
	{
		GPIO_PinAFConfig(GET_HW_CTX(port), pin, type);
	}
}

void gpio_close(int port, int pin)
{
	GPIO_InitTypeDef io;
	
    io.GPIO_Mode  = GPIO_Mode_IN;
	io.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	io.GPIO_OType = GPIO_OType_OD;
	io.GPIO_Speed = GPIO_Speed_100MHz;
	io.GPIO_Pin   = 1<<pin;
	GPIO_Init(GET_HW_CTX(port),&io);
}

int gpio_read(int port, int pin)
{
	return (((GET_HW_CTX(port)->IDR)>>pin)&0x01);
}

void gpio_write(int port, int pin, int value)
{
	if(value)
	{
		GET_HW_CTX(port)->BSRRL = (1<<pin);
	}
	else
	{
		GET_HW_CTX(port)->BSRRH = (1<<pin);
	}
}

void gpio_init(void)
{

}
