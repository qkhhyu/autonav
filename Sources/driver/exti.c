/*
* STM32F4xx EXTIÇý¶¯
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2017.05.26
*/
#include "stm32f4xx.h"
#include "nvic.h"
#include "exti.h"

struct exti_contex
{
	void (*handler)(void);
};

#define GET_CTX(id) (&m_exti_ctx[id])
static struct exti_contex m_exti_ctx[16];

void exti_init(void)
{
	nvic_open(EXTI0_IRQn, 1);
    nvic_open(EXTI1_IRQn, 1);
    nvic_open(EXTI2_IRQn, 1);
    nvic_open(EXTI3_IRQn, 1);
    nvic_open(EXTI4_IRQn, 1);
    nvic_open(EXTI9_5_IRQn, 1);
    nvic_open(EXTI15_10_IRQn, 1);
}

void exti_open(int port, int pin, int mode, void(*handler)(void))
{
	EXTI_InitTypeDef init;
	struct exti_contex *ctx;
	ctx = GET_CTX(pin);
	ctx->handler = handler;
	init.EXTI_Mode = EXTI_Mode_Interrupt;
	init.EXTI_LineCmd = ENABLE;
	init.EXTI_Line = (1 << pin);
	init.EXTI_Trigger = (EXTITrigger_TypeDef)mode;
	EXTI_Init(&init);
    SYSCFG_EXTILineConfig(port, pin);
}

void exti_close(int port, int pin)
{
	EXTI_InitTypeDef init;
	struct exti_contex *ctx;
	ctx = GET_CTX(pin);
	ctx->handler = 0;
	init.EXTI_Mode = EXTI_Mode_Interrupt;
	init.EXTI_LineCmd = DISABLE;
	init.EXTI_Line = (1 << pin);
	init.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&init);
}

static void exti_irq_handler(int line)
{
    struct exti_contex *ctx;
	ctx = GET_CTX(line);
	if(EXTI_GetITStatus(1 << line))
	{
		EXTI_ClearITPendingBit(1 << line);
		if(ctx->handler)
		{
			ctx->handler();
		}
	}
}

void EXTI0_IRQHandler(void)
{
	exti_irq_handler(0);
}

void EXTI1_IRQHandler(void)
{
	exti_irq_handler(1);
}

void EXTI2_IRQHandler(void)
{
	exti_irq_handler(2);
}

void EXTI3_IRQHandler(void)
{
	exti_irq_handler(3);
}

void EXTI4_IRQHandler(void)
{
	exti_irq_handler(4);
}

void EXTI9_5_IRQHandler(void)
{
	exti_irq_handler(5);
	exti_irq_handler(6);
	exti_irq_handler(7);
	exti_irq_handler(8);
	exti_irq_handler(9);
}

void EXTI15_10_IRQHandler(void)
{
	exti_irq_handler(10);
	exti_irq_handler(11);
	exti_irq_handler(12);
	exti_irq_handler(13);
	exti_irq_handler(14);
	exti_irq_handler(15);
}
