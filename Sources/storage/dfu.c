/*
* ¹Ì¼þÉý¼¶´æ´¢Ä£¿é
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
*/
#include <stdint.h>
#include "crc.h"
#include "irom.h"
#include "memmap.h"
#include "dfu.h"

#define DFU_MBR_ADDR	DFUMBR_ADDR
#define DFU_MBR_SIZE    DFUMBR_SIZE
#define DFU_MBR_MASK    0x55534544

struct dfu_mbr
{
    uint32_t mask;
	uint32_t type;
    uint32_t addr;
    uint32_t size;
};

static struct dfu_mbr m_mbr[4];

static void dfu_mbr_set(int id, uint32_t type, uint32_t addr, uint32_t size)
{
	struct dfu_mbr *mbr;
	mbr = &m_mbr[id];
	mbr->mask = DFU_MBR_MASK;
    mbr->type = type;
	mbr->addr = addr;
    mbr->size = size;
}

static void dfu_mbr_load(void)
{
    irom_read(DFU_MBR_ADDR, m_mbr, sizeof(m_mbr));
}

static void dfu_mbr_flush(void)
{
    irom_erase(DFU_MBR_ADDR, DFU_MBR_SIZE);
    irom_write(DFU_MBR_ADDR, m_mbr, sizeof(m_mbr));
}

static int dfu_set_stm32(void *data, uint32_t size)
{
    dfu_mbr_load();
	irom_erase(DFUAPP_ADDR, DFUAPP_SIZE);
	irom_write(DFUAPP_ADDR, data, size);
	dfu_mbr_set(0, DFU_TYPE_APP, DFUAPP_ADDR, size);
	dfu_mbr_flush();
	return 1;
}

static int dfu_set_nrf51(int type, void *data, uint32_t size)
{
	dfu_mbr_load();
    irom_erase(DFUNRF_ADDR, DFUNRF_SIZE);
    irom_write(DFUNRF_ADDR, data, size);
	if(type & DFU_TYPE_NRF1)
	{
		dfu_mbr_set(1, DFU_TYPE_NRF1, DFUNRF_ADDR, size);
	}
	if(type & DFU_TYPE_NRF2)
	{
		//dfu_mbr_set(2, DFU_TYPE_NRF2, DFUNRF_ADDR, size);
	}
	dfu_mbr_flush();
    return 1;
}

int dfu_setup(int type, void *data, uint32_t size)
{
    if(type == DFU_TYPE_APP)
    {
		dfu_set_stm32(data, size);
        return 1;
    }
    if(type & DFU_TYPE_NRF)
    {
		dfu_set_nrf51(type, data, size);
        return 1;
    }
    return 0;
}
