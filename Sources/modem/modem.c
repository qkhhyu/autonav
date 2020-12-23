/*
* MODEM模块管理
* ETH/BM817/EC20
* 负责初始化和启动
* 2019.9.18
*/
#include <string.h>
#include "kernel.h"
#include "app.h"
#include "gpio.h"
#include "log.h"
#include "atcmd.h"
#include "ec20.h"
#include "modem.h"
#include "modem_device.h"

#define GPIO_MODEM_RESET PG,12
#define GPIO_MODEM_POWER PI,6

#define MODULE_NAME "MODEM"

static event_t m_event;
static mutex_t m_mutex;
static char    m_model[64];
static int     m_wait;
static int     m_boot;
static int     m_reboot_count;
static struct modem_device m_device;

//处理应答
static int modem_handle_line(const char *line)
{
	if(m_wait)
	{
		LOG("%s\r\n", line);
		if(strcmp(line, "OK") == 0)
		{
			event_post(m_event);
			return 1;
		}
		if(strncmp(line, "EC20F", 5) == 0)
		{
			strcpy(m_model, "EC20F");
			event_post(m_event);
			return 1;
		}
		if(strncmp(line, "BM817", 5) == 0)
		{
			strcpy(m_model, "BM817");
			event_post(m_event);
			return 1;
		}
	}
	return 0;
}

//发送命令
static int modem_send_cmd(char *cmd, uint32_t timeout)
{
	int ret;
	m_wait = 1;
	event_reset(m_event);
	atcmd_send_line(cmd);
	ret = event_timed_wait(m_event, 1000);
	m_wait = 0;
	return ret;
}

//等待就绪
static int modem_wait_ready(void)
{
	int i;
	int ret;
	for(i=0; i<10; i++)
	{
		sleep(5000);
		LOG("waiting for ready...\r\n");
		ret = modem_send_cmd("AT", 1000);
		if(ret != 0)
		{
			return 1;
		}
	}
	LOG("waiting for ready timeout!\r\n");
	return 0;
}

//等待确定型号
static int modem_wait_model(void)
{
	int i;
	int ret;
	for(i=0; i<10; i++)
	{
		LOG("waiting for model...\r\n");
		ret = modem_send_cmd("AT+GMM", 1000);
		if(ret != 0)
		{
			return 1;
		}
	}
	LOG("waiting for model timeout!\r\n");
	return 0;
}

static void modem_wait_reset(void)
{
	LOG("\r\n");
	LOG("power reset %d times.\r\n", ++m_reboot_count);
	gpio_open(GPIO_MODEM_POWER,GPIO_MODE_OUT,GPIO_OUT_PP);
	gpio_write(GPIO_MODEM_POWER,0);
	
	gpio_open(GPIO_MODEM_RESET,GPIO_MODE_OUT,GPIO_OUT_PP);
	gpio_write(GPIO_MODEM_RESET,0);
	sleep(1000);
	gpio_write(GPIO_MODEM_RESET,1);
	sleep(1000);
}

static void modem_daemon(void *arg)
{
	int ret;
	while(1)
	{
		ret = m_boot ? modem_rssi() : 0;
		appvar.modem_rssi = (ret < 32) ? ret : 0;
		sleep(30000);
	}
}

static int modem_identify(void)
{
	if(strncmp(m_model, "EC20F", 5) == 0)
	{
		ec20_init();
		strcpy(appvar.modem_name, m_model);
		thread_create(modem_daemon,0,10240);
		return 1;
	}
	LOG("invalid modem: %s\r\n", m_model);
	return 0;
}

int modem_init(void)
{
	int ret;
	atcmd_init();
	atcmd_create_handler("", modem_handle_line);
	m_event = event_create();
	m_mutex = mutex_create();
	modem_wait_reset();
	ret = modem_wait_ready();
	if(ret == 0)
	{
		return 0;
	}
	ret = modem_wait_model();
	if(ret == 0)
	{
		return 0;
	}
	ret = modem_identify();
	if(ret == 0)
	{
		return 0;
	}
	ret = modem_boot();
	if(ret == 0)
	{
		return 0;
	}
	m_boot = 1;
	LOG("modem init ok\r\n");
	return 1;
}

int modem_reboot(void)
{
	m_boot = 0;
	modem_wait_reset();
	while(!modem_boot())
	{
		sleep(60000);
	}
	m_boot = 1;
	return 1;
}

void modem_device_bind(struct modem_device *device)
{
	memcpy(&m_device, device, sizeof(struct modem_device));
}

int modem_boot(void)
{
	int ret;
	mutex_lock(m_mutex);
	ret = m_device.boot();
	mutex_unlock(m_mutex);
	return ret;
}

int  modem_rssi(void)
{
	int ret;
	mutex_lock(m_mutex);
	ret = m_device.rssi();
	mutex_unlock(m_mutex);
	return ret;
}

int  modem_socket(void)
{
	int ret;
	mutex_lock(m_mutex);
	ret = m_device.socket_create();
	mutex_unlock(m_mutex);
	return ret;
}

void modem_delete(int sock)
{
	mutex_lock(m_mutex);
	m_device.socket_delete(sock);
	mutex_unlock(m_mutex);
}

int  modem_connect(int sock, const char *addr, int port)
{
	int ret;
	mutex_lock(m_mutex);
	ret = m_device.socket_connect(sock, addr, port);
	mutex_unlock(m_mutex);
	return ret;
}

void modem_close(int sock)
{
	mutex_lock(m_mutex);
	m_device.socket_close(sock);
	mutex_unlock(m_mutex);
}

int  modem_send(int sock, void *data, int len)
{
	int ret;
	mutex_lock(m_mutex);
	ret = m_device.socket_send(sock, data, len);
	mutex_unlock(m_mutex);
	return ret;
}

int  modem_recv(int sock, void *data, int len)
{
	int ret;
	ret = m_device.socket_recv(sock, data, len);
	return ret;
}
