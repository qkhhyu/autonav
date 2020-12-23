/******************************************************************************
* IROM Çý¶¯
* STM32F429ZG 1MB=512KB+512KB
* MAP: 16K+16K+16K+16K+64K+128K*7
* BANK1: 0x08000000 - 0x08100000
* BANK2: 0x08100000 - 0x08200000
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
******************************************************************************/
#include "stm32f4xx.h"
#include "irom.h"

struct sector_table
{
	uint32_t index;
	uint32_t start;
	uint32_t size;
};

static struct sector_table sector_table[]=
{
	FLASH_Sector_0,	 0x08000000, 0x00004000,
	FLASH_Sector_1,	 0x08004000, 0x00004000,
	FLASH_Sector_2,	 0x08008000, 0x00004000,
	FLASH_Sector_3,  0x0800C000, 0x00004000,
	FLASH_Sector_4,  0x08010000, 0x00010000,
	FLASH_Sector_5,  0x08020000, 0x00020000,
	FLASH_Sector_6,  0x08040000, 0x00020000,
	FLASH_Sector_7,  0x08060000, 0x00020000,
	FLASH_Sector_8,  0x08080000, 0x00020000,
	FLASH_Sector_9,  0x080A0000, 0x00020000,
	FLASH_Sector_10, 0x080C0000, 0x00020000,
	FLASH_Sector_11, 0x080E0000, 0x00020000,
};

#define MAX_SECTOR	(sizeof(sector_table)/sizeof(sector_table[0]))

uint32_t irom_erase(uint32_t addr, uint32_t len)
{
	uint32_t i;
	uint32_t end;
	FLASH_Status state;

	end = addr + len;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	for(i=0; i<MAX_SECTOR; i++)
	{
		if(sector_table[i].start < addr)
		{
			continue;
		}
		if(sector_table[i].start >= end)
		{
			FLASH_Lock();
			return 1;
		}
		state = FLASH_EraseSector(sector_table[i].index, VoltageRange_3);
		if(state != FLASH_COMPLETE)
		{
			FLASH_Lock();
			return 0;
		}
	}
	FLASH_Lock();
	return 0;
}

uint32_t irom_write(uint32_t addr, void* data, uint32_t len)
{
	uint32_t i;
	FLASH_Status state;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	for(i=0; i<len; i++)
	{
		state = FLASH_ProgramByte(addr+i, ((uint8_t*)data)[i]);
		if(state != FLASH_COMPLETE)
		{
			break;
		}
	}
	FLASH_Lock();
	return (i==len);
}

uint32_t irom_read(uint32_t addr, void* data, uint32_t len)
{
	uint32_t i;
	for(i=0; i<len; i++)
	{
		((uint8_t*)data)[i] = ((uint8_t*)addr)[i];
	}
	return i;
}

void irom_boot(uint32_t addr)
{
	register uint32_t sp;
	register uint32_t pc;
	sp = *((uint32_t*)(addr + 0));
	pc = *((uint32_t*)(addr + 4));
	__disable_irq();
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, addr - FLASH_BASE);
	__set_MSP(sp);
	((void(*)(void))pc)();
}
