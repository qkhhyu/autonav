/*
* 机器配置信息模块
* 用于保存机器的配置
* 蒋晓岗<kerndev@foxmail.com>
*/
#include <string.h>
#include "kernel.h"
#include "w25qxx.h"
#include "memmap.h"
#include "syscfg.h"

#define SYSCFG_MAGIC			0x4D414304

static void syscfg_default(struct syscfg* cfg)
{
	memset(cfg, 0, sizeof(struct syscfg));
	memset(cfg->device_mac, 0x88, 6);
	strcpy(cfg->device_sn,"1234567890123");
}

void syscfg_read(struct syscfg* cfg)
{
	w25qxx_read(SYSCFG_ADDR, cfg, sizeof(struct syscfg));
	if(cfg->magic != SYSCFG_MAGIC)
	{
		syscfg_default(cfg);
	}
}

void syscfg_write(struct syscfg* cfg)
{
	cfg->magic = SYSCFG_MAGIC;
	w25qxx_erase(SYSCFG_ADDR, SYSCFG_SIZE);
	w25qxx_write(SYSCFG_ADDR, cfg, sizeof(struct syscfg));
}
