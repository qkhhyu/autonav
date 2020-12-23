/*
* ����FTP�ͻ���
* ��½,GET,PUT����
* ������<kerndev@foxmail.com>
* 2017.04.20
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "modem.h"
#include "ftp.h"

#define MODULE_NAME "FTP"

static int  socket_cmd;
static int  socket_data;
static char send_buff[1024];
static char recv_buff[1024];

//����˿ڣ���������
static int ftp_send_command(char *cmd)
{
	int ret;
	LOG("send command: %s\r\n", cmd);
	ret = modem_send(socket_cmd,cmd,(int)strlen(cmd));
	if(ret < 0)
	{
		LOG("failed to send command:%s",cmd);
		return 0;
	}
	return 1;
}

//����˿ڣ�����Ӧ��
static int ftp_recv_respond(char *resp, int len)
{
	int ret;
	int off;
	len -= 1;
	for(off=0;off<len;off+=ret)
	{
		ret = modem_recv(socket_cmd,&resp[off],1);
		if(ret < 0)
		{
			LOG("recv respond error(ret=%d)!\r\n", ret);
			return 0;
		}
		if(resp[off] == '\n')
		{
			break;
		}
	}
	resp[off+1] = 0;
	LOG("respond:%s",resp);
	if(resp[3] != 0x20)
	{
		return ftp_recv_respond(resp, len);
	}
	return atoi(resp);
}

//����FTP������Ϊ����ģʽ�����������ݶ˿�
static int ftp_enter_pasv(char *ipaddr, int *port)
{
	int ret;
	char *find;
	int a,b,c,d;
	int pa,pb;
	ret = ftp_send_command("PASV\r\n");
	if(ret != 1)
	{
		return 0;
	}
	ret = ftp_recv_respond(recv_buff,1024);
	if(ret != 227)
	{
		return 0;
	}
	find = strrchr(recv_buff,'(');
	sscanf(find, "(%d,%d,%d,%d,%d,%d)", &a, &b, &c, &d, &pa, &pb);
	sprintf(ipaddr, "%d.%d.%d.%d", a, b, c, d);
	*port = pa * 256 + pb;
	return 1;
}

//�ϴ��ļ�
int  ftp_upload(char *name, void *buf, int len)
{
	int  ret;
	char ipaddr[32];
	int  port;
	
	//��ѯ���ݵ�ַ
	ret=ftp_enter_pasv(ipaddr,&port);
	if(ret != 1)
	{
		return 0;
	}
	ret=modem_connect(socket_data,ipaddr,port);
	if(ret != 1)
	{
		return 0;
	}
	//׼���ϴ�
	sprintf(send_buff,"STOR %s\r\n",name);
	ret=ftp_send_command(send_buff);
	if(ret != 1)
	{
		return 0;
	}
	ret=ftp_recv_respond(recv_buff,1024);
	if(ret != 150)
	{
		modem_close(socket_data);
		return 0;
	}
	
	//��ʼ�ϴ�
	ret=modem_send(socket_data,buf,len);
	if(ret != len)
	{	
		LOG("tcp send data error!\r\n");
		modem_close(socket_data);
		return 0;
	}
	modem_close(socket_data);

	//�ϴ���ɣ��ȴ���Ӧ
	ret=ftp_recv_respond(recv_buff,1024);
	return (ret==226);
}

//�����ļ�
int  ftp_download(char *name, void *buf, int len)
{
	int   i;
	int   ret;
	char  ipaddr[32];
	int   port;
    
	//��ѯ���ݵ�ַ
	ret = ftp_enter_pasv(ipaddr,&port);
	if(ret != 1)
	{
		return 0;
	}
	//�������ݶ˿�
	ret=modem_connect(socket_data,ipaddr,port);
	if(ret != 1)
	{
		LOG("failed to connect data port\r\n");
		return 0;
	}

	//׼������
	sprintf(send_buff,"RETR %s\r\n",name);
	ret=ftp_send_command(send_buff);
	if(ret != 1)
	{
		return 0;
	}
	ret = ftp_recv_respond(recv_buff,1024);
	if(ret != 150)
	{
		modem_close(socket_data);
		return 0;
	}
	
	//��ʼ����,��ȡ�����ݺ󣬷��������Զ��ر�����
	for(i=0; i<len; i+=ret)
	{
		ret = modem_recv(socket_data, ((char *)buf) + i, len);
		LOG("download %d/%d.\r\n", i + ret, len);
		if(ret < 0)
		{
			LOG("download was interrupted.\r\n");
			break;
		}
	}
	//�������
	LOG("download %d/%d bytes complete.\r\n", i, len);
	modem_close(socket_data);
	ret = ftp_recv_respond(recv_buff,1024);
	return (ret==226);
}

//�����ļ���С
int  ftp_filesize(char* name)
{
	int ret;
	int size;
	sprintf(send_buff,"SIZE %s\r\n",name);
	ret = ftp_send_command(send_buff);
	if(ret != 1)
	{
		return 0;
	}
	ret = ftp_recv_respond(recv_buff,1024);
	if(ret != 213)
	{
		return 0;
	}
	size = atoi(recv_buff+4);
	return size;
}

//��½������
int ftp_login(char* addr, int port, char* username, char* password)
{
	int ret;
	LOG("connect...\r\n");
	ret = modem_connect(socket_cmd, addr, port);
	if(ret != 1)
	{
		LOG("connect server failed!\r\n");
		return 0;
	}
	LOG("connect ok.\r\n");
	ret = ftp_recv_respond(recv_buff,1024);		//�ȴ���ӭ��Ϣ
	if(ret != 220)
	{
		LOG("bad server, ret=%d!\r\n", ret);
		modem_close(socket_cmd);
		return 0;
	}
	
	LOG("login...\r\n");
	sprintf(send_buff,"USER %s\r\n",username);	//����USER
	ret = ftp_send_command(send_buff);
	if(ret != 1)
	{
		modem_close(socket_cmd);
		return 0;
	}
	ret = ftp_recv_respond(recv_buff,1024);
	if(ret != 331)
	{
		modem_close(socket_cmd);
		return 0;
	}
	
	sprintf(send_buff,"PASS %s\r\n",password);	//����PASS
	ret = ftp_send_command(send_buff);
	if(ret != 1)
	{
		modem_close(socket_cmd);
		return 0;
	}
	ret = ftp_recv_respond(recv_buff,1024);
	if(ret != 230)
	{
		modem_close(socket_cmd);
		return 0;
	}
	LOG("login success.\r\n");
	
	ret = ftp_send_command("TYPE I\r\n");	//����Ϊ������ģʽ
	if(ret != 1)
	{
		modem_close(socket_cmd);
		return 0;
	}
	ret = ftp_recv_respond(recv_buff,1024);
	if(ret != 200)
	{
		modem_close(socket_cmd);
		return 0;
	}
	return 1;
}

void ftp_quit(void)
{
	ftp_send_command("QUIT\r\n");
	modem_close(socket_cmd);
}

void ftp_init(void)
{
	socket_cmd = modem_socket();
	socket_data= modem_socket();
}
