#ifndef __MQTT_H
#define __MQTT_H

typedef struct
{
	char    *topic;       //����
	uint32_t topic_size;  //���ⳤ��
	char    *data;        //��Ϣ����
	uint32_t data_size;   //��Ϣ����
	uint16_t pid;         //����
	uint8_t  qos;         //QoS�ȼ�
	uint8_t  retain;      //�Ƿ���
}mqtt_message_t;

typedef struct
{
	char           *client_id;   //�ͻ���ID[����]
	char           *user_name;   //�û���[��ѡ]
	char           *pass_word;   //����[��ѡ]
	mqtt_message_t *will_msg;    //������Ϣ[��ѡ]
	uint16_t        keep_alive;  //�������
	uint8_t         keep_session;//�Ƿ񱣳ֻỰ
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
