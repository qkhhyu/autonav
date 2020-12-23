/*
* 业务平台客户端<自有协议>
*******************************************************************************
* 1.同步服务器时间
* 2.定时发送心跳包
* 3.按需发送数据
*******************************************************************************
* 蒋晓岗<kerndev@foxmail.com>
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "board.h"
#include "time.h"
#include "bizp.h"
#include "bizp_client.h"
#include "cjson.h"

#define LOG_INFO         LOG
#define MAX_SEND_SIZE    30

static event_t m_event;

//-------------------------------------------------------------------------------------------------
static void invoke_cmd_reboot(char *msg_id, cJSON *arg)
{
	int size;
	char reply[256];
	uint32_t tms;
	time_get_timestamp(&tms);
	size = sprintf(reply, "{\"functionId\":reboot,\"messageId\":\"%s\",\"deviceId\":\"%s\",\"timestamp\":%u000,\"success\":true,\"output\":{\"fake\":\"0\"}}", msg_id, appcfg.device_id, tms);
	bizp_reply_run_command(reply, size);
	LOG_INFO("will reboot...\r\n");
	sleep(1000);
	board_reset();
}

static void invoke_cmd_dfu(char *msg_id, cJSON *arg)
{
	int size;
	char reply[256];
	uint32_t tms;
	time_get_timestamp(&tms);
	size = sprintf(reply, "{\"functionId\":dfu,\"messageId\":\"%s\",\"deviceId\":\"%s\",\"timestamp\":%u000,\"success\":true,\"output\":{\"fake\":\"0\"}}", msg_id, appcfg.device_id, tms);
	bizp_reply_run_command(reply, size);
	LOG_INFO("will reboot...\r\n");
	sleep(1000);
	board_reset();
}

static void invoke_cmd_sync_time(char *msg_id, cJSON *arg)
{
	cJSON *item;
	int size;
	char reply[256];
	struct date date;
	uint32_t tms;
	time_get_timestamp(&tms);
	
	item = cJSON_GetObjectItem(arg, "serverSendTime");
	if(item != NULL)
	{
		time_set_timestamp(item->valuedouble / 1000);
		time_get_date(&date);
		LOG_INFO("new time: %d/%d/%d %02d:%02d:%02d\r\n",date.year,date.month,date.day,date.hour,date.minute,date.second);
		event_post(m_event);
	}
	
	size = sprintf(reply, "{\"functionId\":sync_time,\"messageId\":\"%s\",\"deviceId\":\"%s\",\"timestamp\":%u000,\"success\":true,\"output\":{}}", msg_id, appcfg.device_id, tms);
	bizp_reply_run_command(reply, size);
}

static void invoke_command(char *fun, char *msg_id, cJSON *arg)
{
	if(strcmp(fun, "reboot") == 0)
	{
		invoke_cmd_reboot(msg_id, arg);
		return;
	}
	if(strcmp(fun, "sync_time") == 0)
	{
		invoke_cmd_sync_time(msg_id, arg);
		return;
	}
	if(strcmp(fun, "dfu") == 0)
	{
		invoke_cmd_dfu(msg_id, arg);
		return;
	}
	LOG_INFO("invalid command:%s!\r\n", fun);
}

//-------------------------------------------------------------------------------------------------
static void read_property(char *msg_id, char *name)
{
	int size;
	char reply[256];
	uint32_t timestamp;
	time_get_timestamp(&timestamp);
	if(strcmp(name, "version") == 0)
	{
		size = sprintf(reply, "{\"messageId\":\"%s\",\"deviceId\":\"%s\",\"timestamp\":%u000,\"success\":true,\"properties\":{\"version\":\"%s\"}}", msg_id, appcfg.device_id,timestamp,appvar.sw_version);
		bizp_reply_get_property(reply, size);
		return;
	}
	if(strcmp(name, "battery") == 0)
	{
		size = sprintf(reply, "{\"messageId\":\"%s\",\"deviceId\":\"%s\",\"timestamp\":%u000,\"success\":true,\"properties\":{\"battery\":%d}}", msg_id, appcfg.device_id,timestamp,0);
		bizp_reply_get_property(reply, size);
		return;
	}
	LOG_INFO("invalid properties: %s\r\n", name);
}

//获取参数
static void on_get_property(char *json)
{
	cJSON *root;
	cJSON *msg_id;
	cJSON *arg;
	root = cJSON_Parse(json);
	if(root == NULL)
	{
		LOG_INFO("parsm json failed!\r\n");
		return;
	}
	
	msg_id = cJSON_GetObjectItem(root, "messageId");
	if(msg_id == NULL)
	{
		LOG_INFO("parsm messageId failed!\r\n");
		cJSON_Delete(root);
		return;
	}
	
	arg = cJSON_GetObjectItem(root, "properties");
	if(arg == NULL)
	{
		LOG_INFO("parsm properties failed!\r\n");
		cJSON_Delete(root);
		return;
	}
	
	arg = cJSON_GetArrayItem(arg, 0);
	if(arg == NULL)
	{
		cJSON_Delete(root);
		return;
	}
	
	read_property(msg_id->valuestring, arg->valuestring);
	cJSON_Delete(root);
}

//设置参数
static void on_set_property(char *json)
{
	LOG_INFO("unsupport write properties!\r\n");
}

//执行功能
static void on_run_command(char *json)
{
	cJSON *root;
	cJSON *fun_id;
	cJSON *msg_id;
	cJSON *arg;
	root = cJSON_Parse(json);
	if(root == NULL)
	{
		LOG_INFO("parsm json failed!\r\n");
		return;
	}
	
	msg_id = cJSON_GetObjectItem(root, "messageId");
	if(msg_id == NULL)
	{
		LOG_INFO("parsm messageId failed!\r\n");
		cJSON_Delete(root);
		return;
	}
	
	fun_id = cJSON_GetObjectItem(root, "functionId");
	if(fun_id == NULL)
	{
		LOG_INFO("parsm functionId failed!\r\n");
		cJSON_Delete(root);
		return;
	}
	
	arg = cJSON_GetObjectItem(root, "inputs");
	if(arg == NULL)
	{
		LOG_INFO("parsm inputs failed!\r\n");
		cJSON_Delete(root);
		return;
	}
	
	invoke_command(fun_id->valuestring, msg_id->valuestring, arg);
	cJSON_Delete(root);
}

//-------------------------------------------------------------------------------------------------
//等待连接服务器
static void wait_connect(void)
{
	int ret;
	int retry;
	bizp_disconnect();
	for(retry = 0; true; retry++)
	{
		LOG("\r\n");
		LOG_INFO("connect...\r\n");
		ret = bizp_connect();
		if(ret != 1)
		{
            appvar.bizp_online--;
			LOG_INFO("connect failed %d times!\r\n", retry);
			sleep(10000);
			continue;
		}
        appvar.bizp_online += retry;
		LOG_INFO("connect ok.\r\n");
		return;
	}
}

//等待握手和时间同步
static void wait_sync_time(void)
{
	int ret;
    int retry;
	char json[64];
	for(retry = 0; true; retry++)
	{
		LOG("\r\n");
		LOG_INFO("sync time...\r\n");
		ret = sprintf(json, "{\"data\":{\"deviceSendTime\":0}}");
		ret = bizp_send_event("sync_time", json, ret);
		if(ret <= 0)
		{
			appvar.bizp_online--;
			LOG_INFO("sync time failed!\r\n");
			sleep(10000);
			wait_connect();
			continue;
		}
		ret = event_timed_wait(m_event, 10000);
		if(ret == 0)
		{
            appvar.bizp_online--;
			LOG_INFO("sync time timeout!\r\n");
			continue;
		}
		appvar.bizp_online += retry;
		LOG_INFO("sync time ok\r\n");
		return;
	}
}

//等待发送心跳成功
static void wait_send_heart(void)
{
	int ret;
	int retry;
	for(retry = 0; true; retry++)
	{
		LOG("\r\n");
		LOG_INFO("send heart...\r\n");
		ret = bizp_send_heart();
		if(ret == 0)
		{
            appvar.bizp_online--;
			LOG_INFO("send heart failed!\r\n");
			wait_connect();
			continue;
		}
        appvar.bizp_online += retry;
		LOG_INFO("send heart ok.\r\n");
		return;
	}
}

//等待上传数据
static int wait_send_data(void)
{
	return 0;
}

//业务平台主线程
static void bizp_thread_main(void *arg)
{
	int ret;
	int interval;

	wait_connect();
	wait_sync_time();
    wait_send_heart();
    appvar.bizp_online = 1;
	interval = 0;
	while(1)
	{
		ret = wait_send_data();
		if(ret)
		{
			continue;
		}
		sleep(1000);
		interval++;
		if(interval > 30)
		{
			interval = 0;
			wait_send_heart();
		}
	}
}

//初始化
void bizp_client_init(void)
{
	struct bizp_handler handler={on_get_property,on_set_property,on_run_command};
	m_event = event_create();
	bizp_init();
	bizp_bind(&handler);
	thread_create(bizp_thread_main, 0, 10*1024);
}
