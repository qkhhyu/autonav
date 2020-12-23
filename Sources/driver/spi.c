/******************************************************************************
* STM32F4xx Ó²¼þSPIÇý¶¯
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2016.6.12
******************************************************************************/
#include "stm32f4xx.h"
#include "gpio.h"
#include "spi.h"

struct spi_contex
{
	SPI_TypeDef *spi;
	uint32_t clk;
    int gpio_af;
    int gpio_port1;
    int gpio_pin1;
    int gpio_port2;
    int gpio_pin2;
    int gpio_port3;
    int gpio_pin3;
};

#define SPI_CLK         (45000000)
#define	SPI_GET_CTX(id)	(&m_spi_ctx[id])

static struct spi_contex m_spi_ctx[]=
{
    {0},
    {SPI1, 0},
    {SPI2, 0},
    {SPI3, SPI_CLK, GPIO_AF_SPI3, PC, 10, PC, 11, PC, 12},
    {SPI4, SPI_CLK, GPIO_AF_SPI4, PE,  2, PE,  5, PE,  6},
    {SPI5, 0},
    {SPI6, 0},
};

static void spi_gpio_init(int id)
{
	struct spi_contex *ctx;
	ctx = SPI_GET_CTX(id);
	gpio_open(ctx->gpio_port1, ctx->gpio_pin1, GPIO_MODE_AF, ctx->gpio_af);
    gpio_open(ctx->gpio_port2, ctx->gpio_pin2, GPIO_MODE_AF, ctx->gpio_af);
	gpio_open(ctx->gpio_port3, ctx->gpio_pin3, GPIO_MODE_AF, ctx->gpio_af);
}

static uint16_t spi_calc_scaler(struct spi_contex *ctx, uint32_t baudrate)
{
	uint16_t sht;
	uint32_t temp;
	for(sht=0; sht<8; sht++)
	{
		temp = ctx->clk >> (sht+1);
		if(temp <= baudrate)
		{
			break;
		}
	}
	return sht;
}

void spi_open(int id, int baudrate)
{
	uint16_t scaler;
	SPI_InitTypeDef init;
	struct spi_contex *ctx;
	ctx = SPI_GET_CTX(id);
	scaler = spi_calc_scaler(ctx,baudrate);
	init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	init.SPI_Mode = SPI_Mode_Master;
	init.SPI_DataSize = SPI_DataSize_8b;
	init.SPI_CPOL = SPI_CPOL_Low;
	init.SPI_CPHA = SPI_CPHA_1Edge;
	init.SPI_NSS = SPI_NSS_Soft;
	init.SPI_BaudRatePrescaler = scaler << 3;
	init.SPI_FirstBit = SPI_FirstBit_MSB;
	init.SPI_CRCPolynomial = 7;
	SPI_Init(ctx->spi, &init);
	SPI_Cmd(ctx->spi, ENABLE);
}

void spi_close(int id)
{
	struct spi_contex *ctx;
	ctx = SPI_GET_CTX(id);
	SPI_Cmd(ctx->spi, DISABLE);
}

char spi_xfer(int id, char data)
{
    struct spi_contex* ctx;
	ctx = SPI_GET_CTX(id);
	while((ctx->spi->SR & SPI_I2S_FLAG_TXE)==0);                   
    ctx->spi->DR = data;
    while((ctx->spi->SR & SPI_I2S_FLAG_RXNE)==0);
	return ctx->spi->DR;
}

void spi_send(int id, char data)
{
	spi_xfer(id, data);
}

char spi_recv(int id)
{
    return spi_xfer(id, 0xFF);
}

void spi_send_bulk(int id, void *buff, int len)
{
    int i;
    char *ch = buff;
    for(i=0; i<len; i++)
    {
        spi_xfer(id, *ch++);
    }
}

void spi_recv_bulk(int id, void *buff, int len)
{
    int i;
    char *ch = buff;
    for(i=0; i<len; i++)
    {
        *ch++ = spi_xfer(id, 0xFF);
    }
}

void spi_init(void)
{
    //spi_gpio_init(3);
    spi_gpio_init(4);
}
