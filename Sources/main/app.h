#ifndef __APP_H
#define __APP_H
#include "define.h"
#include "syscfg.h"
#include "appcfg.h"
#include "track.h"

#define APP_HW_VER	"ZL-B810"
#define APP_SW_VER	"0.0.1"

//机器运行状态参数
struct appvar
{	
    char   hw_version[64];
    char   sw_version[64];
	int    modem_warn;
	int    modem_rssi;
	char   modem_name[64];
	int    bizp_online;
	float  heading;      //朝向
	struct gps   gps;
	struct dist  dist;
	struct batt  batt[4];//市电,太阳能,电池1,电池2
	struct motor sail[3];//电机1,电机2,舵机
	struct motor feed[2];
	//以下是临时调试参数
	float  pid_config[2][3]; //2组PID参数
	int    dst_course; //目标航向
};

extern struct syscfg syscfg;
extern struct appcfg appcfg;
extern struct appvar appvar;

void app_init(void);
int  app_update(char *name, void *mem,  uint32_t len);

#endif
