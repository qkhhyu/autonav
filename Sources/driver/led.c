/*
* LED
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2017.05.02
*/
#include "gpio.h"
#include "led.h"

#define GPIO_LED_RED		PD,3
#define GPIO_LED_GREEN		PD,4

void led_write(int id, int data)
{
	switch(id)
	{
	case LED_RED:
		gpio_write(GPIO_LED_RED, data);
		break;
	case LED_GREEN:
		gpio_write(GPIO_LED_GREEN, data);
		break;
	default:
		break;
	}
}

void led_open(int id)
{
	led_write(id, 0);
}

void led_close(int id)
{
	led_write(id, 1);
}

void led_init(void)
{

}
