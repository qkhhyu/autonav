/*
* 使用硬件定时器实现精准延时
* 蒋晓岗<kerndev@foxmail.com>
*/
#include "stm32f4xx.h"
#include "delay.h"

#define CLK_MHZ      90
#define TIM_DEV      TIM2
#define TIM_ENABLE() RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE)

void delay_init(void)
{
	TIM_TimeBaseInitTypeDef tim;

	tim.TIM_Prescaler = CLK_MHZ - 1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 0xFFFFFFFF;
	tim.TIM_ClockDivision = 0x0;
	tim.TIM_RepetitionCounter = 0x0;
	
	TIM_ENABLE();
	TIM_TimeBaseInit(TIM_DEV, &tim);
	TIM_Cmd(TIM_DEV, ENABLE);
}

void delay_us(uint32_t us)
{
	uint32_t cnt;
	cnt = TIM_DEV->CNT;
	while((TIM_DEV->CNT - cnt) < us);
}

void delay_ms(uint32_t ms)
{
	while(ms--)
	{
		delay_us(1000);
	}
}
