#ifndef __MQTT_H
#define __MQTT_H

typedef struct
{
	char    *topic;       //主题
	uint32_t topic_size;  //主题长度
	char    *data;        //消息内容
	uint32_t data_size;   //消息长度
	uint16_t pid;         //包号
	uint8_t  qos;         //QoS等级
	uint8_t  retain;      //是否保留
}mqtt_message_t;

typedef struct
{
	char           *client_id;   //客户端ID[必须]
	char           *user_name;   //用户名[可选]
	char           *pass_word;   //密码[可选]
	mqtt_message_t *will_msg;    //遗嘱消息[可选]
	uint16_t        keep_alive;  //心跳间隔
	uint8_t         keep_session;//是否保持会话
}mqtt_client_t;

typedef struct
{
	void (*on_push)(mqtt_message_t *msg);
}mqtt_handler_t;

void mqtt_init(void);
int  mqtt_connect(char *addr, int port, mqtt_client_t *client, mqtt_handler_t *handler);
int  mqtt_disconnect(void);
int  mqtt_publish(mqtt_message_t *msg);
int  mqtt_subscribe(char *topic, uint8_t qos);
int  mqtt_unsubscribe(char *topic);
int  mqtt_ping(void);

#endif
