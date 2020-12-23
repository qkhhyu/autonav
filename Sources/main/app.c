/*
* 应用程序初始化入口
* 蒋晓岗<kerndev@foxmail.com>
* 2016.4.26
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "svc.h"
#include "dfu.h"
#include "gps.h"
#include "hmi.h"
#include "dist.h"
#include "modem.h"
#include "protect.h"
#include "bizp_client.h"

struct syscfg syscfg;
struct appcfg appcfg;
struct appvar appvar;

static void appvar_init(void)
{
	track_init();
	syscfg_read(&syscfg);
	appcfg_read(&appcfg);
	memset(&appvar, 0, sizeof(appvar));
    sprintf(appvar.hw_version, APP_HW_VER);
    sprintf(appvar.sw_version, "APP_MAIN_V%s", APP_SW_VER);
}

static void print_osver(void)
{
	uint32_t ver;
	ver = kernel_version();
	LOG("\r\nKLite Ver:%d.%d.%d\r\n", (ver>>24)&0xFF, (ver>>16)&0xFF, ver&0xFFFF);
}

static void print_reset(void)
{
	uint8_t i;
	uint8_t ret;
	const char* str[8]={"LPWR","WWDG","IWDG","SWRST","POR","HWRST","BOR","NULL"};
	ret = board_reset_reason();
	LOG("RESET REASON:0x%02X( ", ret);
	for(i=0;i<8;i++)
	{
		if(ret & (0x80>>i))
		{
			DBG("%s ", str[i]);
		}
	}
	DBG(")\r\n");
}

static void print_swver(void)
{
	LOG("----------------------------------------------\r\n");
	LOG("sw version:  %s\r\n", appvar.sw_version);
	LOG("hw version:  %s\r\n", appvar.hw_version);
	LOG("device sn:   %s\r\n", syscfg.device_sn);
	LOG("device id:   %s\r\n", appcfg.device_id);
	LOG("bizp server: %s:%d\r\n",appcfg.bizp_addr,appcfg.bizp_port);
    LOG("----------------------------------------------\r\n");
}

static void banner_init(void)
{
	print_osver();
	print_reset();
	print_swver();
}

static void beep_check(void)
{
	beep_open();
	sleep(500);
	beep_close();
}

static void service_init(void)
{
	svc_daemon_init();
	svc_mpu9250_init();
	svc_485_init();
	svc_hmi_init();
	svc_gps_init();
	svc_dist_init();
	svc_auto_init();
	//bizp_client_init();
}

void app_init(void)
{
	beep_check();
	appvar_init();
	banner_init();
	protect_init();
	protect_auth();
	service_init();
}

int app_update(char *name, void *mem, uint32_t len)
{
	int ret;

	if(memcmp(name, "ZL-B810_CPU", 11)==0)
	{
		LOG("app update start...\r\n");
		ret = dfu_setup(DFU_TYPE_APP, mem, len);
		LOG("app update %s\r\n", ret ? "ok" : "failed!");
		return ret;
	}
    if(memcmp(name, "ZL-B810_HMI", 11)==0)
    {
        LOG("nrf update start...\r\n");
		ret = dfu_setup(DFU_TYPE_NRF, mem, len);
		LOG("nrf update ret=%d\r\n",ret);
		return ret;
    }
	LOG("unknown file type!\r\n");
	return 0;
}

