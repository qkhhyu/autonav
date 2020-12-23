/*
* EC20全网通模块
* 波特率115200
* TCP有12个通道,1-11
* 蒋晓岗 <kerndev@foxmail.com>
* 2019.08.06 创建
*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "kernel.h"
#include "gpio.h"
#include "app.h"
#include "log.h"
#include "fifo.h"
#include "binconv.h"
#include "atcmd.h"
#include "ec20.h"
#include "modem_device.h"

#define	MODULE_NAME "EC20"

struct mobile_contex
{
    int   mnc;
	int   all;
	int   gprs;
	int   umts;
	int   lte;
    char *apn;
    char *apn_usr;
    char *apn_pwd;
};

struct device_contex
{
	int  boot;
	int  resp;
	int  cpin;
	int  rssi;
	int  mnc;
	int  nrs;
	int  open;
	event_t resp_event;
	event_t open_event;
};

struct socket_contex
{
	int     pipe;
	int     state;
	fifo_t  fifo;
	event_t recv_event;
};

#define SOCKET_STATE_INVALID		0
#define SOCKET_STATE_CLOSED			1
#define SOCKET_STATE_CONNECTED		2

#define MAX_SOCKET_ID				5
#define GET_SOCKET_CTX(x)			( (((x)>0)&&((x)<MAX_SOCKET_ID)) ? &m_socket_contex[x] : NULL )

static char                   m_buffer_send[4096];
static char                   m_buffer_recv[4096];
static struct device_contex   m_device_contex;
static struct socket_contex   m_socket_contex[MAX_SOCKET_ID];
static struct mobile_contex   m_mobile_contex[]=
{
	/* mnc  all   gprs  umts  lte   apn      usr pwd */
	{46000, 0x00, 0x01, 0x04, 0x03, "cmnet", "", ""},
    {46001, 0x00, 0x01, 0x02, 0x03, "3gnet", "", ""},
    {46002, 0x00, 0x01, 0x04, 0x03, "cmnet", "", ""},
    {46003, 0x00, 0x06, 0x07, 0x03, "ctnet", "ctnet@mycdma.cn", "vnet.mobi"},
    {46005, 0x00, 0x06, 0x07, 0x03, "ctnet", "ctnet@mycdma.cn", "vnet.mobi"},
    {46006, 0x00, 0x01, 0x02, 0x03, "3gnet", "", ""},
    {46007, 0x00, 0x01, 0x04, 0x03, "cmnet", "", ""},
    {46011, 0x00, 0x06, 0x07, 0x03, "ctnet", "ctnet@mycdma.cn", "vnet.mobi"},
};

//查找移动网络上下文
static struct mobile_contex * ec20_find_mobile(void)
{
    int i;
    int count;
	struct mobile_contex *contex;
    count = sizeof(m_mobile_contex) / sizeof(m_mobile_contex[0]);
    for(i=0; i<count; i++)
    {
		contex = &m_mobile_contex[i];
        if(contex->mnc == m_device_contex.mnc)
        {
            return contex;
        }
    }
	return NULL;
}

//收到正确应答
//OK
static int atcmd_handler_ok(const char *line)
{
	LOG("%s\r\n", line);
	m_device_contex.resp = 1;
	event_post(m_device_contex.resp_event);
	return 1;
}

//收到错误应答
//ERROR
static int atcmd_handler_error(const char *line)
{
	LOG("%s\r\n", line);
	m_device_contex.resp = 0;
	event_post(m_device_contex.resp_event);
	return 1;
}

//收到版本号
//Revision: EC20CEHCLGR06A03M1G
static int atcmd_handler_rev(const char *line)
{
	char *find;
	find = strstr(line, "_AUD");
	appvar.modem_warn = (find != NULL);
	return 0;
}

//收到信号强度
//+CSQ: 2
static int atcmd_handler_csq(const char *line)
{
	LOG("%s\r\n", line);
	m_device_contex.rssi = atoi(line + 5);
	return 1;
}

//收到CPIN
//+CPIN: READY
static int atcmd_handler_cpin(const char *line)
{
	LOG("%s\r\n", line);
	if(strncmp(line + 7, "READY", 5) == 0)
	{
		m_device_contex.cpin = 1;
	}
	return 1;
}

//收到CIMI
//460030892578353
static int atcmd_handler_cimi(const char *line)
{
	char str[8];
	LOG("%s\r\n", line);
	if(strncmp(line, "46", 2) == 0)
	{
		strncpy(str, line, 5);
		m_device_contex.mnc = atoi(str);
		return 1;
	}
	return 0;
}

//收到CREG(0,1,3,5)
//+CREG: 0, 1
static int atcmd_handler_creg(const char *line)
{
	int nrs;
	LOG("%s\r\n", line);
	nrs = atoi(line + 9);
	nrs = (nrs == 3) ? 1 : nrs;
	m_device_contex.nrs = (nrs == 1);
	return 1;
}

//收到PPP连接
//+QIACT:1,10.228.220.122
//OK
static int atcmd_handler_qiact(const char *line)
{
	LOG("%s\r\n", line);
	return 1;
}

//收到QIURC事件
//+QIURC: "closed",1 连接断开
//+QIURC: "recv",1,<size><CR><LF><data>
static int atcmd_handler_qiurc(const char *line)
{
	int pipe;
	int size;
	struct socket_contex *ctx;
	
	LOG("%s\r\n", line);
	if(line[9] == 'c')
	{
		pipe = line[17] - '0';
		ctx = GET_SOCKET_CTX(pipe);
		if(ctx != NULL)
		{
			ctx->state = SOCKET_STATE_CLOSED;
			event_post(ctx->recv_event);
		}
		return 1;
	}
	if(line[9] == 'r')
	{
		pipe = line[15] - '0';
		size = atoi(&line[17]);
		ctx = GET_SOCKET_CTX(pipe);
		if(ctx != NULL)
		{
			atcmd_recv_data(m_buffer_recv, size);
			fifo_write(ctx->fifo, m_buffer_recv, size);
			event_post(ctx->recv_event);
		}
	}
	return 1;
}

//收到连接状态
//+QIOPEN: 1,567
static int atcmd_handler_qiopen(const char *line)
{
	LOG("%s\r\n", line);
	m_device_contex.open = (line[11] == '0');
	event_post(m_device_contex.open_event);
	return 1;
}

//发送命令，并等待应答
static int ec20_send_cmd(char *cmd, int timeout)
{
	int  ret;
	atcmd_enter_mutex();
	event_reset(m_device_contex.resp_event);
	//LOG("%s\r\n", cmd);
	atcmd_send_line(cmd);
	ret = event_timed_wait(m_device_contex.resp_event, timeout);
	atcmd_leave_mutex();
	return ret;
}

//等待通信
static int ec20_wait_ready(void)
{
	int i;
	int ret;
	for(i=0; i<10; i++)
	{
		LOG("waiting for ready...\r\n");
		m_device_contex.resp = 0;
		ret = ec20_send_cmd("AT", 1000);
		if((ret != 0) && (m_device_contex.resp != 0))
		{
			LOG("ready\r\n");
			ec20_send_cmd("ATI", 1000);
			ec20_send_cmd("ATE0", 1000);
			ec20_send_cmd("AT+CREG=0", 1000);
			ec20_send_cmd("AT+CMEE=2", 1000);
			sleep(1000);
			return 1;
		}
		LOG("ret=%d, resp=%d\r\n", ret, m_device_contex.resp);
		sleep(5000);
	}
	LOG("ready timeout!\r\n");
	return 0;
}

//等待SIM卡
static int ec20_wait_cpin(void)
{
	int i;
	int ret;
	
	for(i=0; i<10; i++)
	{
		LOG("waiting for cpin...\r\n");
		m_device_contex.cpin = 0;
		ret = ec20_send_cmd("AT+CPIN?", 1000);
		if((ret != 0) && (m_device_contex.cpin != 0))
		{
			LOG("cpin ok\r\n");
			return 1;
		}
		sleep(5000);
	}
	LOG("cpin timeout!\r\n");
	return 0;
}

//等待查询CIMI
static int ec20_wait_cimi(void)
{
	int i;
	int ret;
	
	for(i=0; i<10; i++)
	{
		LOG("waiting for cimi...\r\n");
		m_device_contex.mnc = 0;
		ret = ec20_send_cmd("AT+CIMI", 1000);
		if((ret != 0) && (m_device_contex.mnc != 0))
		{
			LOG("cimi ok\r\n");
			return 1;
		}
		sleep(5000);
	}
	LOG("cimi timeout!\r\n");
	return 0;
}

//设置网络模式
static int ec20_set_netmode(int mode)
{
	int i;
	int ret;

	for(i=0; i<10; i++)
	{
		LOG("waiting for set netmode...\r\n");
		m_device_contex.resp = 0;
        sprintf(m_buffer_send, "AT+QCFG=\"nwscanmode\",%d", mode);
		ret = ec20_send_cmd(m_buffer_send, 1000);
		if((ret != 0) && (m_device_contex.resp != 0))
		{
			LOG("set netmode ok\r\n");
			ec20_send_cmd("AT+QCFG=\"nwscanmode\"", 1000);
			sleep(5000);	//等待设置生效
			return 1;
		}
		sleep(5000);
	}
	LOG("set netmode timeout!\r\n");
	return 0;
}

//设置APN
static int ec20_set_apn(char *apn, char *user, char *pwd)
{
	int i;
	int ret;
	
	for(i=0; i<10; i++)
	{
		LOG("waiting for set apn...\r\n");
		m_device_contex.resp = 0;
        sprintf(m_buffer_send, "AT+QICSGP=1,1,\"%s\",\"%s\",\"%s\"", apn, user, pwd);
		ret = ec20_send_cmd(m_buffer_send, 1000);
		if((ret != 0) && (m_device_contex.resp != 0))
		{
			LOG("set apn ok\r\n", apn);
			ec20_send_cmd("AT+QICSGP=1", 1000);
			return 1;
		}
		sleep(5000);
	}
	LOG("set apn timeout!\r\n", apn);
	return 0;
}


//等待初始化设置
static int ec20_wait_setup(struct mobile_contex *mobile)
{
	int ret;
	int netmode;
	
	if(strcmp(appcfg.modem, "auto") == 0)
	{
		netmode = mobile->all;
	}
	else if(strcmp(appcfg.modem, "gprs") == 0)
	{
		netmode = mobile->gprs;
	}
	else if(strcmp(appcfg.modem, "3g") == 0)
	{
		netmode = mobile->umts;
	}
	else if(strcmp(appcfg.modem, "lte") == 0)
	{
		netmode = mobile->lte;
	}
	else
	{
		netmode = -1;
		LOG("unknown netmode: %s\r\n", appcfg.modem);
	}
	
	if(netmode >= 0)
	{
		LOG("netmode: %s(%d)\r\n", appcfg.modem, netmode);
		ret = ec20_set_netmode(netmode);
		if(ret == 0)
		{
			LOG("netmode setup failed!\r\n");
			return 0;
		}
	}
	
	if(appcfg.apn_name[0] != 0)
	{
		LOG("user apn: [%s],[%s],[%s]\r\n", appcfg.apn_name, appcfg.apn_user, appcfg.apn_pass);
		ret = ec20_set_apn(appcfg.apn_name, appcfg.apn_user, appcfg.apn_pass);
		if(ret == 0)
		{
			LOG("apn setup failed!\r\n");
			return 0;
		}
	}
	else
	{
		LOG("auto apn: [%s],[%s],[%s]\r\n", mobile->apn, mobile->apn_usr, mobile->apn_pwd);
		ret = ec20_set_apn(mobile->apn, mobile->apn_usr, mobile->apn_pwd);
		if(ret == 0)
		{
			LOG("apn setup failed!\r\n");
			return 0;
		}
	}
	
	return 1;
}

//等待注册网络
static int ec20_wait_creg(void)
{
	int i;
	int ret;

	for(i=0; i<10; i++)
	{
		LOG("waiting for creg...\r\n");
		m_device_contex.nrs = 0;
		ret = ec20_send_cmd("AT+CREG?", 1000);
		if((ret != 0) && (m_device_contex.nrs != 0))
		{
			LOG("creg ok\r\n");
			ec20_send_cmd("AT+COPS?", 1000);
            return 1;
		}
		sleep(5000);
	}
	LOG("creg timeout!\r\n");
	return 0;
}

//等待GPRS启动
static int ec20_wait_ipcall(void)
{
	int i;
	int ret;
	
	for(i=0; i<10; i++)
	{
		LOG("waiting for ipcall...\r\n");
		ret = ec20_send_cmd("AT+QIDEACT=1", 30000);
		ret = ec20_send_cmd("AT+QIACT=1", 30000);
		if((ret != 0) && (m_device_contex.resp != 0))
		{
			LOG("ipcall ok\r\n");
			ec20_send_cmd("AT+QIACT?", 1000);
			ec20_send_cmd("AT+QNWINFO", 1000);	//查询网络信息
			return 1;
		}
		sleep(5000);
	}
	LOG("ipcall timeout!\r\n");
	return 0;
}


//描述: 启动无线模块
static int ec20_boot(void)
{
	int ret;
	struct mobile_contex *mobile;
	
	LOG("\r\n");
	LOG("boot...\r\n");
	m_device_contex.boot = 0;
	m_device_contex.open = 0;
	ret = ec20_wait_ready();
	if(ret != 1)
	{
		return 0;
	}

	ret = ec20_wait_cpin();
	if(ret != 1)
	{
		return 0;
	}
	
	ret = ec20_wait_cimi();
	if(ret != 1)
	{
		return 0;
	}
	
	mobile = ec20_find_mobile();
	if(mobile == NULL)
	{
		return 0;
	}
	
	ret = ec20_wait_setup(mobile);
	if(ret != 1)
	{
		return 0;
	}
	
	ret = ec20_wait_creg();
	if(ret != 1)
	{
		return 0;
	}
	
	ret = ec20_wait_ipcall();
	if(ret != 1)
	{
		return 0;
	}
	m_device_contex.boot = 1;
	LOG("boot ok.\r\n\r\n");
	return 1;
}

//描述: 读取无电线模块信号强度
static int ec20_rssi(void)
{
	m_device_contex.rssi = 0;
	ec20_send_cmd("AT+CSQ", 1000);
	if(m_device_contex.rssi != 0)
	{
		return m_device_contex.rssi;
	}
	LOG("RSSI:error\r\n");
	return 0;
}

//创建socket
static int ec20_socket_create(void)
{
	int i;
	struct socket_contex *ctx;

	for(i=1; i<MAX_SOCKET_ID; i++)
	{
		ctx = GET_SOCKET_CTX(i);
		if(ctx->state == SOCKET_STATE_INVALID)
		{
			ctx->pipe = i;
			ctx->state = SOCKET_STATE_CLOSED;
			ctx->fifo = fifo_create(64 * 1024);
			ctx->recv_event= event_create();
			return i;
		}
	}
	return -1;
}

//删除socket
static void ec20_socket_delete(int sock)
{
	struct socket_contex *ctx;
	ctx = GET_SOCKET_CTX(sock);
	if(ctx != NULL)
	{
		fifo_delete(ctx->fifo);
		event_delete(ctx->recv_event);
		ctx->fifo = NULL;
		ctx->recv_event= NULL;
		ctx->state = SOCKET_STATE_INVALID;
	}
}

//连接服务器
static int ec20_socket_connect(int sock, const char *addr, int port)
{
	int  ret;
	struct socket_contex *ctx;
	
	LOG("connect: sock=%d, addr=%s:%d\r\n", sock, addr, port);
	if(m_device_contex.boot == 0)
	{
		LOG("device busy!\r\n");
		return -1;
	}
	
	ctx = GET_SOCKET_CTX(sock);
	if(ctx == NULL)
	{
		LOG("invalid socket!\r\n");
		return -1;
	}

	event_reset(m_device_contex.open_event);
	m_device_contex.open = 0;
	m_device_contex.resp = 0;
	sprintf(m_buffer_send, "AT+QIOPEN=1,%d,\"TCP\",\"%s\",%d,0,1", ctx->pipe, addr, port);
	ret = ec20_send_cmd(m_buffer_send, 1000);
	if(ret == 0)
	{
		LOG("connect cmd timeout!\r\n");
		return 0;
	}
	if(m_device_contex.resp == 0)
	{
		LOG("connect cmd error!\r\n");
		return 0;
	}
	ret = event_timed_wait(m_device_contex.open_event, 20000);
	if(ret == 0)
	{
		LOG("connect timeout!\r\n");
		sprintf(m_buffer_send, "AT+QICLOSE=%d\r", ctx->pipe);
		ec20_send_cmd(m_buffer_send, 10000);
		event_timed_wait(m_device_contex.open_event, 1000);	//CLOSE之后，模块还会返回一次+QIOPEN:0,567表示已关闭
		return 0;
	}
	if(m_device_contex.open == 0)
	{
		LOG("connect error!\r\n");
		sprintf(m_buffer_send, "AT+QICLOSE=%d\r", ctx->pipe);
		ec20_send_cmd(m_buffer_send, 10000);
		return 0;
	}
	fifo_clear(ctx->fifo);
	ctx->state = SOCKET_STATE_CONNECTED;
	LOG("connect ok\r\n");
	return 1;
}

//断开与主机的连接
static void ec20_socket_close(int sock)
{
	int ret;
	struct socket_contex *ctx;
	
	LOG("close: sock=%d\r\n", sock);
	if(m_device_contex.boot == 0)
	{
		LOG("device busy!\r\n");
		return;
	}
	
	ctx = GET_SOCKET_CTX(sock);
	if(ctx == NULL)
	{
		LOG("invalid socket!\r\n");
		return;
	}
	m_device_contex.resp = 0;
	sprintf(m_buffer_send, "AT+QICLOSE=%d\r", ctx->pipe);
	ret = ec20_send_cmd(m_buffer_send, 20000);
	ctx->state = SOCKET_STATE_CLOSED;
	event_post(ctx->recv_event);
	if(ret == 0)
	{
		LOG("close timeout!\r\n");
		return;
	}
	if(m_device_contex.resp == 0)
	{
		LOG("close error!\r\n");
		return;
	}
	LOG("close ok\r\n");
}


//向已连接的主机发送数据
static int ec20_socket_send(int sock, void *buf, int len)
{
	int ret;
	int rst;
	int cur;
	char *bin;
	struct socket_contex *ctx;
	
	LOG("send: socket=%d, size=%d\r\n", sock, len);
	ctx = GET_SOCKET_CTX(sock);
	if(ctx == NULL)
	{
		LOG("invalid socket!\r\n");
		return -1;
	}
	if(ctx->state != SOCKET_STATE_CONNECTED)
	{
		LOG("socket closed!\r\n");
		return 0;
	}
	
	bin = (char *)buf;
	rst = len;
	while(rst > 0)
	{
		m_device_contex.resp = 0;
		cur = (rst < 256) ? rst : 256;
		ret = sprintf(m_buffer_send, "AT+QISENDEX=%d,\"", ctx->pipe);
		bin_to_hex(bin, &m_buffer_send[ret], cur);
		strcpy(&m_buffer_send[ret + cur * 2], "\"");
		ret = ec20_send_cmd(m_buffer_send, 20000);
		if(ret == 0)
		{
			LOG("send timeout!\r\n");
			return 0;
		}
		if(m_device_contex.resp == 0)
		{
			LOG("send error!\r\n");
			return 0;
		}
		bin += cur;
		rst -= cur;
	}
	LOG("send ok\r\n");
	return len;
}


//从已连接的主机读取数据
static int ec20_socket_recv(int sock, void *buf, int len)
{
	int ret;
	struct socket_contex *ctx;
	
	ctx = GET_SOCKET_CTX(sock);
	if(ctx == NULL)
	{
		LOG("invalid socket\r\n");
		return -1;
	}
	while(1)
	{
		ret = fifo_read(ctx->fifo, buf, len);
		if(ret > 0)
		{
			return ret;
		}
		if(ctx->state != SOCKET_STATE_CONNECTED)
		{
			return -1;
		}
		event_wait(ctx->recv_event);
	}
}

static void ec20_modem_register(void)
{
	struct modem_device device=
	{
		ec20_boot,
		ec20_rssi,
		ec20_socket_create,
		ec20_socket_delete,
		ec20_socket_connect,
		ec20_socket_close,
		ec20_socket_send,
		ec20_socket_recv,
	};
	modem_device_bind(&device);
}

static void ec20_contex_init(void)
{
	int i;
	struct socket_contex *ctx;
	m_device_contex.open_event = event_create();
	m_device_contex.resp_event = event_create();
	for(i=0; i<MAX_SOCKET_ID; i++)
	{
		ctx = GET_SOCKET_CTX(i);
		ctx->pipe  = i;
		ctx->state = SOCKET_STATE_INVALID;
		ctx->fifo  = NULL;
		ctx->recv_event = NULL;
	}
}

static void ec20_handler_init(void)
{
	//注册应答处理函数
	atcmd_create_handler("OK", atcmd_handler_ok);
	atcmd_create_handler("ERROR", atcmd_handler_error);
	atcmd_create_handler("SEND OK", atcmd_handler_ok);
	atcmd_create_handler("SEND FAIL", atcmd_handler_error);
	atcmd_create_handler("+CME ERROR", atcmd_handler_error);
	atcmd_create_handler("+CMS ERROR", atcmd_handler_error);
	atcmd_create_handler("Revision:", atcmd_handler_rev);
	atcmd_create_handler("46", atcmd_handler_cimi);
	atcmd_create_handler("+CSQ:", atcmd_handler_csq);
	atcmd_create_handler("+CPIN:", atcmd_handler_cpin);
	atcmd_create_handler("+CREG:", atcmd_handler_creg);
	atcmd_create_handler("+QIACT:", atcmd_handler_qiact);
	atcmd_create_handler("+QIURC:", atcmd_handler_qiurc);
	atcmd_create_handler("+QIOPEN:", atcmd_handler_qiopen);
}

//模块初始化
void ec20_init(void)
{
	LOG("init...\r\n");
	ec20_contex_init();
	ec20_handler_init();
	ec20_modem_register();
	LOG("init ok.\r\n");
}
