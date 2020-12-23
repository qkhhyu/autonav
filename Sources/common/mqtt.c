/*
* 基于socket的简易MQTT客户端
* 蒋晓岗<kerndev@foxmail.com>
* 2020.07.13  创建
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kernel.h"
#include "log.h"
#include "modem.h"
#include "mqtt.h"

#define MODULE_NAME     "MQTT"

#define MAX_SEND_SIZE   (64 * 1024)
#define MAX_RECV_SIZE   (64 * 1024)

static int      m_socket;
static event_t  m_event;
static uint8_t *m_send_buf;
static uint8_t *m_recv_buf;
static uint8_t *m_rest_buf;
static int      m_recv_len;
static int      m_rest_len;
static mqtt_handler_t m_handler;

//MQTT长度编码,返回编码字节数
static int encode_size(uint32_t size, uint8_t *buf)
{
	int i;
	for(i=0; i<4; i++)
	{
		buf[i] = size & 0x7F;
		size >>= 7;
		if(size > 0)
		{
			buf[i] |= 0x80;
			continue;
		}
		return i + 1;
	}
	return 0;
}

//MQTT长度解码
static int decode_size(uint8_t *buf)
{
	int i;
	int size = 0;
	for(i=0; i<4; i++)
	{
		size += (buf[i] & 0x7F) << (i * 7);
		if(buf[i] < 0x80)
		{
			return size;
		}
	}
	return 0;
}

//发送报文
static int  mqtt_send_packet(uint8_t cmd, uint8_t *data, uint32_t size)
{
	int ret;
	uint8_t fixed[8];
	fixed[0] = cmd;
	ret = encode_size(size, &fixed[1]);
	if(ret == 0)
	{
		LOG_INFO("invalid size\r\n");
		return 0;
	}
	
	//LOG_INFO("send:\r\n");
	//LOG_DUMP(fixed, ret + 1);
	ret = modem_send(m_socket, fixed, ret + 1);
	if(ret <= 0)
	{
		LOG_INFO("send head failed!\r\n");
		return -1;
	}
	
	if(data)
	{
		//LOG_DUMP(data, size);
		ret = modem_send(m_socket, data, size);
		if(ret <= 0)
		{
			LOG_INFO("send data failed!\r\n");
			return -1;
		}
	}
	return 1;
}

//发送连接报文
static int mqtt_send_connect(mqtt_client_t *client)
{
	uint8_t flag;
	uint8_t *data;
	uint32_t data_size;
	size_t   size;
	
	flag = 0;
	flag |= client->user_name ? 0x80 : 0x00;
	flag |= client->pass_word ? 0x40 : 0x00;
	flag |= client->keep_session ? 0x00 : 0x02;
	if(client->will_msg)
	{
		flag |= 0x04;
		flag |= client->will_msg->retain ? 0x20 : 0x00;
		flag |= client->will_msg->qos << 3;
	}
	
	data = m_send_buf;
	data_size = 0;
	
	//可变头部
	data[0] = 0x00;//协议名称
	data[1] = 0x04;
	data[2] = 'M';
	data[3] = 'Q';
	data[4] = 'T';
	data[5] = 'T';
	data[6] = 0x04;//协议级别
	data[7] = flag;//连接标志
	data[8] = client->keep_alive >> 8; //保持连接
	data[9] = client->keep_alive;
	data_size += 10;
	data += 10;
	
	//载荷数据
	size = strlen(client->client_id);
	data[0] = size >> 8;
	data[1] = size;
	memcpy(&data[2], client->client_id, size);
	data_size += size + 2;
	data += size + 2;
	
	if(client->will_msg)
	{
		//遗嘱标题
		size = client->will_msg->topic_size;
		data[0] = size >> 8;
		data[1] = size;
		memcpy(&data[2], client->will_msg->topic, size);
		data_size += size + 2;
		data += size + 2;
		
		//遗嘱数据
		size = client->will_msg->data_size;
		data[0] = size >> 8;
		data[1] = size;
		memcpy(&data[2], client->will_msg->data, size);
		data_size += size + 2;
		data += size + 2;
	}
	
	if(client->user_name)
	{
		//用户名
		size = strlen(client->user_name);
		data[0] = size >> 8;
		data[1] = size;
		memcpy(&data[2], client->user_name, size);
		data_size += size + 2;
		data += size + 2;
	}
	
	if(client->pass_word)
	{
		//密码
		size = strlen(client->pass_word);
		data[0] = size >> 8;
		data[1] = size;
		memcpy(&data[2], client->pass_word, size);
		data_size += size + 2;
		data += size + 2;
	}
	
	return mqtt_send_packet(0x10, m_send_buf, data_size);
}

//发送发布报文
static int mqtt_send_publish(mqtt_message_t *msg, uint8_t dup)
{
	uint8_t flag;
	uint8_t *data;
	uint32_t data_size;
	size_t   size;
	
	flag = 0;
	flag |= dup ? 0x08 : 0x00;
	flag |= msg->qos << 1;
	flag |= msg->retain ? 0x01 : 0x00;
	
	data = m_send_buf;
	data_size = 0;
	
	size = msg->topic_size;
	data[0] = size >> 8;
	data[1] = size;
	memcpy(&data[2], msg->topic, size);
	data += size + 2;
	data_size += size + 2;
	
	if(msg->qos > 0)
	{
		data[0] = msg->pid >> 8;
		data[1] = msg->pid;
		data += 2;
		data_size += 2;
	}
	
	memcpy(data, msg->data, msg->data_size);
	data_size += msg->data_size;
	
	return mqtt_send_packet(0x30 | flag, m_send_buf, data_size);
}

//发送PUBACK
static int  mqtt_send_puback(uint16_t pid)
{
	uint8_t *data;
	uint32_t data_size;
	
	data = m_send_buf;
	data[0] = pid >> 8;
	data[1] = pid;
	data_size = 2;
	return mqtt_send_packet(0x40, m_send_buf, data_size);
}

#if 0

//发送PUBREC
static int  mqtt_send_pubrec(uint16_t pid)
{
	uint8_t *data;
	uint32_t data_size;
	
	data = m_send_buf;
	data[0] = pid >> 8;
	data[1] = pid;
	data_size = 2;
	return mqtt_send_packet(0x50, m_send_buf, data_size);
}

//发送PUBREL
static int  mqtt_send_pubrel(uint16_t pid)
{
	uint8_t *data;
	uint32_t data_size;
	
	data = m_send_buf;
	data[0] = pid >> 8;
	data[1] = pid;
	data_size = 2;
	return mqtt_send_packet(0x62, m_send_buf, data_size);
}

//发送PUBCOMP
static int  mqtt_send_pubcomp(uint16_t pid)
{
	uint8_t *data;
	uint32_t data_size;
	
	data = m_send_buf;
	data[0] = pid >> 8;
	data[1] = pid;
	data_size = 2;
	return mqtt_send_packet(0x70, m_send_buf, data_size);
}

#endif

//发送SUBSCRIBE
static int  mqtt_send_subscribe(char *topic, uint8_t qos)
{
	uint8_t *data;
	uint32_t data_size;
	size_t   size;
	
	data = m_send_buf;
	data[0] = 0x00;
	data[1] = 0x0A;
	data += 2;
	data_size = 2;
	
	size = strlen(topic);
	data[0] = size >> 8;
	data[1] = size;
	memcpy(&data[2], topic, size);
	data += size + 2;
	data_size += size + 2;
	
	data[0] = qos;
	data += 1;
	data_size += 1;
	return mqtt_send_packet(0x82, m_send_buf, data_size);
}

//发送UNSUBSCRIBE
static int  mqtt_send_unsubscribe(char *topic)
{
	uint8_t *data;
	uint32_t data_size;
	size_t   size;
	
	data = m_send_buf;
	data[0] = 0x00;
	data[1] = 0x0A;
	data += 2;
	data_size = 2;
	
	size = strlen(topic);
	data[0] = size >> 8;
	data[1] = size;
	memcpy(&data[2], topic, size);
	data += size + 2;
	data_size += size + 2;
	
	return mqtt_send_packet(0xA2, m_send_buf, data_size);
}

//发送PING
static int  mqtt_send_ping(void)
{
	return mqtt_send_packet(0xC0, NULL, 0);
}

//发送DISCONNECT
static int  mqtt_send_disconnect(void)
{
	return mqtt_send_packet(0xE0, NULL, 0);
}

//-------------------------------------------------------------------------------------------------
static void on_recv_connack(uint8_t *data, int size)
{
	event_post(m_event);
}

static void on_recv_public(uint8_t qos, uint8_t *data, int size)
{
	mqtt_message_t msg;
	memset(&msg, 0, sizeof(msg));
	msg.qos = qos;
	msg.retain = 0;
	msg.topic = (char*)&data[2];
	msg.topic_size = (data[0] << 8) | data[1];
	data += msg.topic_size + 2;
	size -= msg.topic_size + 2;
	if(qos == 1)
	{
		msg.pid = (data[0] << 8) | data[1];
		data += 2;
		size -= 2;
		mqtt_send_puback(msg.pid);
	}
	msg.data = (char*)&data[0];
	msg.data_size = size;
	if(m_handler.on_push)
	{
		m_handler.on_push(&msg);
	}
}

static void on_recv_puback(uint8_t *data, int size)
{
	event_post(m_event);
}

static void on_recv_suback(uint8_t *data, int size)
{
	event_post(m_event);
}

static void on_recv_unsuback(uint8_t *data, int size)
{
	event_post(m_event);
}

static void on_recv_pingresp(uint8_t *data, int size)
{
	event_post(m_event);
}

static void mqtt_proc_packet(uint8_t cmd, uint8_t *data, int size)
{
	if((cmd & 0xF0) == 0x20)
	{
		on_recv_connack(data, size);
		return;
	}
	if((cmd & 0xF0) == 0x30)
	{
		on_recv_public((cmd >> 1) & 0x03, data, size);
		return;
	}
	if((cmd & 0xF0) == 0x40)
	{
		on_recv_puback(data, size);
		return;
	}
	if((cmd & 0xF0) == 0x90)
	{
		on_recv_suback(data, size);
		return;
	}
	if((cmd & 0xF0) == 0xB0)
	{
		on_recv_unsuback(data, size);
		return;
	}
	if((cmd & 0xF0) == 0xD0)
	{
		on_recv_pingresp(data, size);
		return;
	}
	LOG_INFO("unsupports cmd:0x%02X!\r\n", cmd);
}

static void mqtt_recv_packet(uint8_t *data, int size)
{
	int i;
	uint8_t ch;
	for(i=0; i<size; i++)
	{
		ch = data[i];
		if(m_rest_len == 0)
		{
			m_recv_buf[m_recv_len++] = ch;
			if(m_recv_len > 1)
			{
				if(ch < 0x80)
				{
					m_rest_len = decode_size(&m_recv_buf[1]);
					m_rest_buf = &m_recv_buf[m_recv_len];
					m_recv_len = 0;
					if(m_recv_len == m_rest_len)
					{
						mqtt_proc_packet(m_recv_buf[0], m_rest_buf, m_rest_len);
						m_recv_len = 0;
						m_rest_len = 0;
					}
					continue;
				}
			}
			if(m_recv_len > 5)
			{
				LOG_DUMP("invalid data:", m_recv_buf, m_recv_len);
				m_recv_len = 0;
			}
		}
		else
		{
			m_rest_buf[m_recv_len++] = ch;
			if(m_recv_len == m_rest_len)
			{
				mqtt_proc_packet(m_recv_buf[0], m_rest_buf, m_rest_len);
				m_recv_len = 0;
				m_rest_len = 0;
			}
		}
	}
}

static void mqtt_recv_thread(void *arg)
{
	int ret;
	uint8_t data[1024];
	while(1)
	{
		ret = modem_recv(m_socket, data, 1024);
		if(ret > 0)
		{
			mqtt_recv_packet(data, ret);
			continue;
		}
		sleep(1000);
	}
}

//-------------------------------------------------------------------------------------------------
static int mqtt_wait_conack(void)
{
	int ret;
	ret = event_timed_wait(m_event, 10000);
	if(ret == 0)
	{
		return -1;
	}
	return m_rest_buf[1];
}

static int mqtt_wait_puback(void)
{
	int ret;
	ret = event_timed_wait(m_event, 10000);
	if(ret == 0)
	{
		return 0;
	}
	return 1;
}

static int mqtt_wait_suback(void)
{
	int ret;
	ret = event_timed_wait(m_event, 10000);
	if(ret == 0)
	{
		return -1;
	}
	return m_rest_buf[1];
}

static int mqtt_wait_unsuback(void)
{
	int ret;
	ret = event_timed_wait(m_event, 10000);
	if(ret == 0)
	{
		return 0;
	}
	return 1;
}

static int mqtt_wait_pingresp(void)
{
	int ret;
	ret = event_timed_wait(m_event, 10000);
	if(ret == 0)
	{
		return 0;
	}
	return 1;
}
//-------------------------------------------------------------------------------------------------
void mqtt_init(void)
{
	m_socket = modem_socket();
	m_event  = event_create();
	m_send_buf = heap_alloc(MAX_SEND_SIZE);
	m_recv_buf = heap_alloc(MAX_RECV_SIZE);
	thread_create(mqtt_recv_thread, NULL, 10240);
}

int  mqtt_connect(char *addr, int port, mqtt_client_t *client, mqtt_handler_t *handler)
{
	int ret;
	m_handler = *handler;
	ret = modem_connect(m_socket, addr, port);
	if(ret <= 0)
	{
		LOG_INFO("connect failed!\r\n");
		return -1;
	}
	ret = mqtt_send_connect(client);
	if(ret <= 0)
	{
		LOG_INFO("send connect failed!\r\n");
		return -1;
	}
	ret = mqtt_wait_conack();
	if(ret < 0)
	{
		LOG_INFO("wait connack failed!\r\n");
		return -1;
	}
	LOG_INFO("connect ret=%02X\r\n", ret);
	return ret;
}

int  mqtt_disconnect(void)
{
	mqtt_send_disconnect();
	modem_close(m_socket);
	return 1;
}

int  mqtt_publish(mqtt_message_t *msg)
{
	int ret;
	ret = mqtt_send_publish(msg, 0);
	if(ret < 0)
	{
		LOG_INFO("mqtt_public: send pkt error!\r\n");
		return 0;
	}
	if(msg->qos == 1)
	{
		ret = mqtt_wait_puback();
		if(ret == 0)
		{
			LOG_INFO("mqtt_public: wait ack error!\r\n");
			return 0;
		}
	}
	LOG_INFO("mqtt_public ok\r\n");
	return 1;
}

int  mqtt_subscribe(char *topic, uint8_t qos)
{
	int ret;
	ret = mqtt_send_subscribe(topic, qos);
	if(ret < 0)
	{
		LOG_INFO("mqtt_subscribe: send pkt error!\r\n");
		return 0;
	}
	ret = mqtt_wait_suback();
	if(ret < 0)
	{
		LOG_INFO("mqtt_subscribe: wait ack error!\r\n");
		return 0;
	}
	LOG_INFO("mqtt_subscribe ok\r\n");
	return 1;
}

int  mqtt_unsubscribe(char *topic)
{
	int ret;
	ret = mqtt_send_unsubscribe(topic);
	if(ret < 0)
	{
		LOG_INFO("mqtt_send_unsubscribe: send pkt error!\r\n");
		return 0;
	}
	ret = mqtt_wait_unsuback();
	if(ret < 0)
	{
		LOG_INFO("mqtt_send_unsubscribe: wait ack error!\r\n");
		return 0;
	}
	LOG_INFO("mqtt_send_unsubscribe ok\r\n");
	return 1;
}

int  mqtt_ping(void)
{
	int ret;
	ret = mqtt_send_ping();
	if(ret < 0)
	{
		LOG_INFO("mqtt_ping: send pkt error!\r\n");
		return 0;
	}
	ret = mqtt_wait_pingresp();
	if(ret == 0)
	{
		LOG_INFO("mqtt_ping: wait ack error!\r\n");
		return 0;
	}
	LOG_INFO("mqtt_ping ok\r\n");
	return 1;
}
