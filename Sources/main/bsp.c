/*
* 板级资源初始化
* 蒋晓岗<kerndev@foxmail.com>
* 2016.4.26
*/
#include "bsp.h"

void bsp_init(void)
{
	delay_init();
	i2c_init();
	spi_init();
	time_init();
	beep_init();
	lm75_init();
	w25qxx_init();
	mpu9250_init();
}
