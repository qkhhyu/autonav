/*
* 物联网平台客户端协议
* 协议文档: MQTT
* 蒋晓岗<kerndev@foxmail.com>
* 2020.07.20 创建
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "mqtt.h"
#include "binconv.h"
#include "bizp.h"

#define LOG_INFO LOG

struct bizp_contex
{
	int     socket;
	event_t event;
	char    resp[1024];
    uint8_t wait_cmd;
	int     pkt_size;
	int     pkt_recv;
	uint8_t pkt_data[1024];
	struct bizp_handler handler;
};

static struct bizp_contex m_contex;

//解析主题路径
static int parsm_topic(char *topic, int level, char *value, int max_size)
{
	int i;
	int curr_level;
	curr_level = 0;
	*value = 0;
	
	for(curr_level = 0; curr_level < level; topic++)
	{
		if(*topic == 0)
		{
			return 0;
		}
		if(*topic == '/')
		{
			curr_level++;
		}
	}
	
	for(i=0; i<max_size; i++)
	{
		value[i] = *topic++;
		if(value[i] == '/')
		{
			value[i] = 0;
			return i;
		}
		if(value[i] == 0)
		{
			return i;
		}
	}
	value[i-1] = 0;
	return i-1;
}

static void proc_message(char *topic, char *payload)
{
	char value[32];
	struct bizp_handler *fun;
	fun = &m_contex.handler;
	
	parsm_topic(topic, 3, value, 30);
	if(strcmp(value, "function") == 0)
	{
		fun->on_run_command(payload);
		return;
	}
	if(strcmp(value, "properties") == 0)
	{
		parsm_topic(topic, 4, value, 30);
		if(strcmp(value, "read") == 0)
		{
			fun->on_get_property(payload);
			return;
		}
		if(strcmp(value, "write") == 0)
		{
			fun->on_set_property(payload);
			return;
		}
	}
	LOG_INFO("invalid topic: %s!\r\n", topic);
}

static void on_mqtt_push(mqtt_message_t *msg)
{
	char topic[256];
	char payload[1024];

	LOG_INFO("on_mqtt_push\r\n");
	memset(topic, 0, 256);
	memcpy(topic, msg->topic, msg->topic_size);
	LOG_INFO("topic: %s\r\n", topic);
	
	memset(payload, 0, 1024);
	memcpy(payload, msg->data, msg->data_size);
	LOG_INFO("msg: %s\r\n", payload);
	
	proc_message(topic, payload);
}

//连接
int bizp_connect(void)
{
	int ret;
	char topic[64];
	mqtt_client_t cli;
	mqtt_handler_t hdl={on_mqtt_push};

	cli.client_id = appcfg.device_id;
	cli.user_name = "admin";
	cli.pass_word = "admin";
	cli.will_msg  = NULL;
	cli.keep_alive= 60;
	cli.keep_session = 0;
	ret = mqtt_connect(appcfg.bizp_addr, appcfg.bizp_port, &cli, &hdl);
	if(ret != 0)
	{
		return 0;
	}
	sprintf(topic, "/ZL-B810/%s/#", appcfg.device_id);
	LOG_INFO("sub:%s\r\n", topic);
	ret = mqtt_subscribe(topic, 0);
	if(ret != 1)
	{
		LOG_INFO("subscribe %s failed!\r\n", topic);
		return 0;
	}
	return 1;
}

int bizp_disconnect(void)
{
	return mqtt_disconnect();
}

//心跳
int bizp_send_heart(void)
{
	return mqtt_ping();
}

int  bizp_send_event(char *event, char *json, int size)
{
	int ret;
	char topic[64];
	mqtt_message_t msg;
	msg.topic = topic;
	msg.topic_size = sprintf(topic, "/ZL-B810/%s/event/%s", appcfg.device_id, event);
	msg.data = json;
	msg.data_size = size;
	msg.qos = 1;
	msg.pid = 0xff;
	msg.retain = 0;
	LOG_INFO("pub:%s\r\nmsg:%s\r\n", topic, msg.data);
	ret = mqtt_publish(&msg);
	return ret;
}

int  bizp_reply_get_property(char *json, int size)
{
	int ret;
	char topic[64];
	mqtt_message_t msg;
	msg.topic = topic;
	msg.topic_size = sprintf(topic, "/ZL-B810/%s/properties/read/reply", appcfg.device_id);
	msg.data = json;
	msg.data_size = size;
	msg.qos = 0;
	msg.pid = 0xff;
	msg.retain = 0;
	LOG_INFO("pub:%s\r\nmsg:%s\r\n", topic, msg.data);
	ret = mqtt_publish(&msg);
	return ret;
}

int  bizp_reply_set_property(char *json, int size)
{
	int ret;
	char topic[64];
	mqtt_message_t msg;
	msg.topic = topic;
	msg.topic_size = sprintf(topic, "/ZL-B810/%s/properties/write/reply", appcfg.device_id);
	msg.data = json;
	msg.data_size = size;
	msg.qos = 0;
	msg.pid = 0xff;
	msg.retain = 0;
	LOG_INFO("pub:%s\r\nmsg:%s\r\n", topic, msg.data);
	ret = mqtt_publish(&msg);
	return ret;
}

int  bizp_reply_run_command(char *json, int size)
{
	int ret;
	char topic[64];
	mqtt_message_t msg;
	msg.topic = topic;
	msg.topic_size = sprintf(topic, "/ZL-B810/%s/function/invoke/reply", appcfg.device_id);
	msg.data = json;
	msg.data_size = size;
	msg.qos = 0;
	msg.pid = 0xff;
	msg.retain = 0;
	LOG_INFO("pub:%s\r\nmsg:%s\r\n", topic, msg.data);
	ret = mqtt_publish(&msg);
	return ret;	
}

void bizp_bind(struct bizp_handler *handler)
{
	memcpy(&m_contex.handler, handler, sizeof(struct bizp_handler));
}

void bizp_init(void)
{
	memset(&m_contex, 0, sizeof(m_contex));
	m_contex.event = event_create();
	mqtt_init();
}
