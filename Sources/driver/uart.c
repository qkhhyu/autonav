/******************************************************************************
* STM32F4xx串口驱动
* USART1,2,3,4,5,6,7,8
* 蒋晓岗<kerndev@foxmail.com>
* 2016.6.12
******************************************************************************/
#include "stm32f4xx.h"
#include "gpio.h"
#include "nvic.h"
#include "uart.h"

struct uart_sw_contex
{
    char *buff;
	int   in;
	int   out;
	int   size;
	int   ovf;
};

struct uart_hw_contex
{
    USART_TypeDef *uart;
    int irq;
    int gpio_af;
    int gpio_port1;
    int gpio_pin1;
    int gpio_port2;
    int gpio_pin2;
};


#define GET_HW_CTX(id)  (&m_uart_hw_ctx[id])
#define GET_SW_CTX(id)  (&m_uart_sw_ctx[id])

static struct uart_sw_contex m_uart_sw_ctx[9];
static struct uart_hw_contex m_uart_hw_ctx[9]=
{
    {0},
    {USART1, USART1_IRQn, GPIO_AF_USART1, PA,  9, PA, 10},
    {USART2, USART2_IRQn, GPIO_AF_USART2, PA,  2, PA,  3},
    {USART3, USART3_IRQn, GPIO_AF_USART3, PB, 10, PB, 11},
    {UART4,  UART4_IRQn,  GPIO_AF_UART4,  PA,  0, PA,  1},
    {UART5,  UART5_IRQn,  GPIO_AF_UART5,  0},
    {USART6, USART6_IRQn, GPIO_AF_USART6, PG,  9, PG, 14},
    {UART7,  UART7_IRQn,  GPIO_AF_UART7,  PF,  6, PF,  7},
    {UART8,  UART8_IRQn,  GPIO_AF_UART8,  0},
};

static void uart_hw_init(int id)
{
    struct uart_hw_contex *ctx;
    ctx = GET_HW_CTX(id);
	gpio_open(ctx->gpio_port1, ctx->gpio_pin1, GPIO_MODE_AF, ctx->gpio_af);
    gpio_open(ctx->gpio_port2, ctx->gpio_pin2, GPIO_MODE_AF, ctx->gpio_af);
    nvic_open(ctx->irq, 0);
}

static void uart_sw_init(int id, void *buffer, int buffer_size)
{
    struct uart_sw_contex *ctx;
    ctx = GET_SW_CTX(id);
    ctx->buff = buffer;
	ctx->size = buffer_size;
	ctx->in   = 0;
	ctx->out  = 0;
	ctx->ovf  = 0;
}

void uart_init(int id, void* buff, int buff_size)
{
    uart_hw_init(id);
    uart_sw_init(id, buff, buff_size);
}

void uart_open(int id, int baudrate, int parity)
{
	USART_InitTypeDef init;
	struct uart_hw_contex *ctx;
	
	switch(parity)
	{
	case ODDPARITY:
		init.USART_Parity = USART_Parity_Odd;
		init.USART_WordLength = USART_WordLength_9b;
		break;
	case EVENPARITY:
		init.USART_Parity = USART_Parity_Even;
		init.USART_WordLength = USART_WordLength_9b;
		break;
	case NOPARITY:
	default:
		init.USART_Parity = USART_Parity_No;
		init.USART_WordLength = USART_WordLength_8b;
		break;
	}
	init.USART_BaudRate = baudrate;
	init.USART_StopBits = USART_StopBits_1;
	init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	ctx = GET_HW_CTX(id);
	USART_Init(ctx->uart, &init);
	USART_ITConfig(ctx->uart, USART_IT_RXNE, ENABLE);
	USART_Cmd(ctx->uart, ENABLE);
}

void uart_close(int id)
{
	struct uart_hw_contex *ctx;
	ctx = GET_HW_CTX(id);
	USART_ITConfig(ctx->uart, USART_IT_RXNE, DISABLE);
	USART_Cmd(ctx->uart, DISABLE);
}

void uart_write(int id, void *buf, int len)
{
	int i;
	struct uart_hw_contex *ctx;
	ctx = GET_HW_CTX(id);
    for(i=0; i<len; i++)
    {
        while(!USART_GetFlagStatus(ctx->uart, USART_FLAG_TXE));
        USART_SendData(ctx->uart, ((char *)buf)[i]);
        while(!USART_GetFlagStatus(ctx->uart, USART_FLAG_TC));
    }
}

int uart_read(int id, void *buf, int len)
{
	int i;
	int end;
	struct uart_sw_contex* ctx;
	
	ctx = GET_SW_CTX(id);
	end = ctx->in;
	for(i=0; i<len; i++)
	{
		if(ctx->out == end)
		{
			break;
		}
		((char *)buf)[i] = ctx->buff[ctx->out];
		ctx->out++;
		if(ctx->out == ctx->size)
		{
			ctx->out = 0;
		}
	}
	return i;
}

int uart_overflow(int id)
{
	struct uart_sw_contex *sw;
	sw = GET_SW_CTX(id);
	if(sw->ovf > 0)
	{
		sw->ovf = 0;
		return 1;
	}
	return 0;
}

void uart_clear(int id)
{
	struct uart_hw_contex *hw;
    struct uart_sw_contex *sw;
	hw = GET_HW_CTX(id);
    sw = GET_SW_CTX(id);
	USART_ITConfig(hw->uart, USART_IT_RXNE, DISABLE);
	sw->in  = 0;
	sw->out = 0;
	sw->ovf = 0;
	USART_ITConfig(hw->uart, USART_IT_RXNE, ENABLE);
}

//中断接收数据
static void uart_irq_handler(int id)
{
	char data;
	int  next;
	struct uart_hw_contex *hw;
    struct uart_sw_contex *sw;
    hw = GET_HW_CTX(id);
    sw = GET_SW_CTX(id);
    
	//UART硬件OVERFLOW
	if(USART_GetFlagStatus(hw->uart, USART_FLAG_ORE) != RESET)
	{
		USART_ReceiveData(hw->uart);
		USART_ClearFlag(hw->uart, USART_FLAG_ORE);
		sw->ovf++;
		return;
	}
	if(USART_GetITStatus(hw->uart, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(hw->uart, USART_IT_RXNE);
		data = USART_ReceiveData(hw->uart);
		next = sw->in + 1;
		if(next >= sw->size)
		{
			next = 0;
		}
		if(next == sw->out)
		{
			sw->ovf++;
			return;
		}
		sw->buff[sw->in] = data;
		sw->in = next;
	}
}

//UART接收中断处理
void USART1_IRQHandler(void)
{
	uart_irq_handler(1);
}

void USART2_IRQHandler(void)
{
	uart_irq_handler(2);
}

void USART3_IRQHandler(void)
{
	uart_irq_handler(3);
}

void UART4_IRQHandler(void)
{
	uart_irq_handler(4);
}

void UART5_IRQHandler(void)
{
	uart_irq_handler(5);
}

void USART6_IRQHandler(void)
{
	uart_irq_handler(6);
}

void UART7_IRQHandler(void)
{
	uart_irq_handler(7);
}

void UART8_IRQHandler(void)
{
	uart_irq_handler(8);
}
