/*
* W25QXX驱动程序
* SPI时钟最大50MHz
* 1 page  = 256Byte(最小编程单位,3ms)
* 1 sector= 16 page = 4KB(最小擦除单位,100ms)
* 1 block = 16 sector = 64KB
* 1 chip  = 256 block = 16MB
*/
#include "bsp.h"
#include "spi.h"
#include "gpio.h"
#include "w25qxx.h"

//SPI接口
#define SPI_PORT			4
#define SPI_NSS				PE,4
#define SPI_SPEED			10000000

//芯片ID
#define CHIP_ID				0x00EF4018
#define CHIP_PAGE_SIZE		256
#define CHIP_SECTOR_SIZE	4096

//指令表
#define CMD_WRITESR			0x01
#define CMD_PAGEPROG		0x02
#define CMD_READDATA		0x03
#define CMD_WRITEDIS		0x04
#define CMD_READSR			0x05
#define CMD_WRITEEN			0x06
#define CMD_ERASE			0x20
#define CMD_CHIPID			0x9F

#define w25qxx_select()		gpio_write(SPI_NSS, 0)
#define w25qxx_deselect()	gpio_write(SPI_NSS, 1)

static void w25qxx_send(uint8_t* data, uint32_t len)
{
	uint32_t i;
	for(i=0; i<len; i++)
	{
		spi_send(SPI_PORT, *data++);
	}
}

static void w25qxx_recv(uint8_t* data, uint32_t len)
{
	uint32_t i;
	for(i=0; i<len; i++)
	{
		*data++ = spi_recv(SPI_PORT);
	}
}

static void w25qxx_wait_busy(void)
{
	uint8_t sr;
	uint8_t cmd;
	sr  = 0x01;
	cmd = CMD_READSR;
	while(sr & 0x01)
	{
		w25qxx_select();
		w25qxx_send(&cmd, 1);
		w25qxx_recv(&sr, 1);
		w25qxx_deselect();
	};
}

static void w25qxx_write_enable(void)
{
	uint8_t cmd;
	cmd = CMD_WRITEEN;
	w25qxx_select();
	w25qxx_send(&cmd, 1);
	w25qxx_deselect();
}

//256字节页编程
static void w25qxx_write_page(uint32_t addr, uint8_t* buf, uint32_t len)
{
	uint8_t cmd[4];
	len = len<CHIP_PAGE_SIZE?len:CHIP_PAGE_SIZE;
	cmd[0] = CMD_PAGEPROG;
	cmd[1] = (addr>>16)&0xFF;
	cmd[2] = (addr>>8)&0xFF;
	cmd[3] = (addr&0xFF);
	w25qxx_select();
	w25qxx_send(cmd, 4);
	w25qxx_send(buf, len);
	w25qxx_deselect();
}

//擦除扇区
static void w25qxx_erase_sector(uint32_t addr)
{
	uint8_t cmd[4];
	cmd[0] = CMD_ERASE;
	cmd[1] = (addr>>16)&0xFF;
	cmd[2] = (addr>>8)&0xFF;
	cmd[3] = (addr&0xFF);
	w25qxx_select();
	w25qxx_send(cmd, 4);
	w25qxx_deselect();
}

//擦除数据
uint32_t w25qxx_erase(uint32_t addr, uint32_t len)
{
	uint32_t i;
	for(i=0; i<len; i+=CHIP_SECTOR_SIZE)
	{
		w25qxx_write_enable();
		w25qxx_erase_sector(addr);
		w25qxx_wait_busy();
		addr += CHIP_SECTOR_SIZE;
	}
	return i;
}


//读数据
uint32_t w25qxx_read(uint32_t addr, void* buf, uint32_t len)
{
	uint8_t cmd[4];
	cmd[0] = CMD_READDATA;
	cmd[1] = (addr>>16)&0xFF;
	cmd[2] = (addr>>8)&0xFF;
	cmd[3] = (addr&0xFF);
	w25qxx_select();
	w25qxx_send(cmd, 4);
	w25qxx_recv(buf, len);
	w25qxx_deselect();
	return len;
}

//写数据
uint32_t w25qxx_write(uint32_t addr, void* buf, uint32_t len)
{
	uint32_t ret;
	uint32_t temp;
	uint8_t* data;
	
	ret  = len;
	data = buf;

	while(len>0)
	{
		temp = (len<CHIP_PAGE_SIZE) ? len : CHIP_PAGE_SIZE;
		w25qxx_write_enable();
		w25qxx_write_page(addr, data, temp);
		w25qxx_wait_busy();
		data += temp;
		addr += temp;
		len  -= temp;
	}
	return ret;
}

uint32_t w25qxx_chip_id(void)
{
	uint8_t cmd;
	uint8_t buf[4];
	uint32_t id;
	cmd = CMD_CHIPID;
	w25qxx_select();
	w25qxx_send(&cmd, 1);
	w25qxx_recv(buf, 4);
	w25qxx_deselect();
	id = buf[0]<<16;
	id|= buf[1]<<8;
	id|= buf[2];
	return id;
}

void w25qxx_init(void)
{
	spi_open(SPI_PORT, SPI_SPEED);
	gpio_open(SPI_NSS, GPIO_MODE_OUT, GPIO_OUT_PP);
	while(w25qxx_chip_id() != CHIP_ID);
}
