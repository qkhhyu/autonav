/******************************************************************************
* 应用配置信息
* 蒋晓岗<kerndev@foxmail.com>
******************************************************************************/
#include <string.h>
#include "kernel.h"
#include "app.h"
#include "time.h"
#include "w25qxx.h"
#include "memmap.h"
#include "appcfg.h"

#define APPCFG_MAGIC 0x415050FF

static void appcfg_default(struct appcfg *cfg)
{
	memset(cfg, 0, sizeof(struct appcfg));
	strcpy(cfg->device_id,"00000000");
	strcpy(cfg->modem, "auto");
	strcpy(cfg->bizp_addr,"113.57.172.27");
	cfg->bizp_port = 1883;
	strcpy(cfg->bizp_user, "admin");
	strcpy(cfg->bizp_pass, "admin");
	
	cfg->compass_offset = 0;
}

void appcfg_read(struct appcfg* cfg)
{
	w25qxx_read(APPCFG_ADDR, cfg, sizeof(struct appcfg));
	if(cfg->magic != APPCFG_MAGIC)
	{
		appcfg_default(cfg);
	}
}

void appcfg_write(struct appcfg *cfg)
{
	cfg->magic = APPCFG_MAGIC;
	w25qxx_erase(APPCFG_ADDR, APPCFG_SIZE);
	w25qxx_write(APPCFG_ADDR, cfg, sizeof(struct appcfg));
}

void appcfg_reset(struct appcfg *cfg)
{
	appcfg_default(cfg);
	appcfg_write(cfg);
}
