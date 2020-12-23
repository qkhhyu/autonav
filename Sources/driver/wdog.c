/*
* STM32看门狗驱动
* 蒋晓岗<kerndev@foxmail.com>
*/
#include "stm32f4xx.h"
#include "wdog.h"

void wdog_init(void)
{
	RCC_LSICmd(ENABLE);
	while(!RCC_GetFlagStatus(RCC_FLAG_LSIRDY));
	
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256);	//40K/256=6.4ms
	IWDG_SetReload(2000);					//大约10秒
	IWDG_ReloadCounter();
    IWDG_Enable();
}

void wdog_feed(void)
{
	IWDG_ReloadCounter();
}

