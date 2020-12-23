/*
* AT命令解释引擎
* 作者: 蒋晓岗<kerndev@foxmail.com>
* 2016.3.24: 优化AT解析函数
* 2016.6.27  精简代码
* 2017.04.20 优化代码命名
* 2019.08.06 重构代码
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "uart.h"
#include "list.h"
#include "atcmd.h"

#define UART_PORT 6
#define UART_BAUD 115200
#define UART_BUFF 32*1024
#define LINE_SIZE 4096

#define MODULE_NAME "ATCMD"

struct atcmd_handler
{
	struct atcmd_handler *prev;
	struct atcmd_handler *next;
	char keyword[32];
	int  keyword_length;
	int(*handler)(const char *);
};

struct atcmd_list
{
	struct atcmd_handler *head;
	struct atcmd_handler *tail;
};

static struct atcmd_list m_list;
static mutex_t           m_list_mutex;
static mutex_t           m_send_mutex;
static char             *m_line_recv;

//处理
static void atcmd_handle_line(const char *line)
{
	struct atcmd_handler *node;
	mutex_lock(m_list_mutex);
	for(node = m_list.head; node != NULL; node = node->next)
	{
		if(strncmp(line, node->keyword, node->keyword_length) != 0)
		{
			continue;
		}
		if(node->handler(line) != 0)
		{
			mutex_unlock(m_list_mutex);
			return;
		}
	}
	LOG("%s\r\n", line);
	mutex_unlock(m_list_mutex);
}

//接收AT数据，并解析
static void atcmd_thread_recv(void *data)
{
	int   ret;
	thread_set_priority(thread_self(), THREAD_PRIORITY_LOW);
	while(1)
	{
		ret = atcmd_recv_line(m_line_recv, LINE_SIZE);
		if(ret > 2)
		{
			atcmd_handle_line(m_line_recv);
		}
	}
}

//接收一行数据
int atcmd_recv_line(char *line, int len)
{
	int  cnt;
	char chr;
	cnt = 0;
	for(cnt = 1; cnt < len; cnt++)
	{
		while(uart_read(UART_PORT, &chr, 1) == 0)
		{
			sleep(20);
		}
		if(chr == 0x0A)
		{
			break;
		}
		*line++ = (chr != 0x0D) ? chr : 0x00;
	}
	return cnt;
}

//接收指定长度数据
void atcmd_recv_data(void *data, int len)
{
	char *ptr;
	int   ret;
	ptr = (char *)data;
	while(len)
	{
		ret = uart_read(UART_PORT, ptr, len);
		if(ret == 0)
		{
			sleep(20);
			continue;
		}
		len -= ret;
		ptr += ret;
	}
}

//发送一行
void atcmd_send_line(char *line)
{
	char ch;
	for(ch = 0; ch != 0x0D; line++)
	{
		ch = (*line != 0) ? *line : 0x0D;
		uart_write(UART_PORT, &ch, 1);
	}
}

//发送数据
void atcmd_send_data(void *data, int len)
{
	uart_write(UART_PORT, data, len);
}


//注册处理函数
void atcmd_create_handler(const char *keyword, int (*handler)(const char *))
{
	struct atcmd_handler *node;
	node = heap_alloc(sizeof(struct atcmd_handler));
	if(node == NULL)
	{
		LOG("create handler failed: %s\r\n", keyword);
		return;
	}
	node->handler = handler;
	node->keyword_length = strlen(keyword);;
	strcpy(node->keyword, keyword);
	mutex_lock(m_list_mutex);
	list_append(&m_list, node);
	mutex_unlock(m_list_mutex);
}

//注销处理函数
void atcmd_delete_handler(const char *keyword, int (*handler)(const char *))
{
	struct atcmd_handler *node;
	mutex_lock(m_list_mutex);
	for(node = m_list.head; node != NULL; node = node->next)
	{
		if(handler != node->handler)
		{
			continue;
		}
		if(strcmp(keyword, node->keyword) == 0)
		{
			list_remove(&m_list, node);
			heap_free(node);
			break;
		}
	}
	mutex_unlock(m_list_mutex);
}

void atcmd_enter_mutex(void)
{
	mutex_lock(m_send_mutex);
}

void atcmd_leave_mutex(void)
{
	mutex_unlock(m_send_mutex);
}

void atcmd_init(void)
{
	m_list_mutex = mutex_create();
	m_send_mutex = mutex_create();
	m_line_recv  = heap_alloc(LINE_SIZE);
	list_init(&m_list);
	uart_init(UART_PORT, heap_alloc(UART_BUFF), UART_BUFF);
	uart_open(UART_PORT, UART_BAUD, 0);
	thread_create(atcmd_thread_recv, 0, 10*1024);
}
