#ifndef __APP_H
#define __APP_H
#include "define.h"
#include "syscfg.h"
#include "appcfg.h"
#include "track.h"

#define APP_HW_VER	"ZL-B810"
#define APP_SW_VER	"0.0.1"

//��������״̬����
struct appvar
{	
    char   hw_version[64];
    char   sw_version[64];
	int    modem_warn;
	int    modem_rssi;
	char   modem_name[64];
	int    bizp_online;
	float  heading;      //����
	struct gps   gps;
	struct dist  dist;
	struct batt  batt[4];//�е�,̫����,���1,���2
	struct motor sail[3];//���1,���2,���
	struct motor feed[2];
	//��������ʱ���Բ���
	float  pid_config[2][3]; //2��PID����
	int    dst_course; //Ŀ�꺽��
};

extern struct syscfg syscfg;
extern struct appcfg appcfg;
extern struct appvar appvar;

void app_init(void);
int  app_update(char *name, void *mem,  uint32_t len);

#endif
