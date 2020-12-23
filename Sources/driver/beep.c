/*
* BEEP
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2016.6.13
*/
#include "gpio.h"
#include "beep.h"

#define BEEP_GPIO	PI,7

void beep_open(void)
{
	gpio_write(BEEP_GPIO, 1);
}

void beep_close(void)
{
	gpio_write(BEEP_GPIO, 0);
}

void beep_init(void)
{
	gpio_open(BEEP_GPIO, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(BEEP_GPIO, 0);
}
