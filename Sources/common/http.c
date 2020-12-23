/*
* 基于socket的简易HTTP客户端
* 蒋晓岗<kerndev@foxmail.com>
* 2016.4.18  创建
* 2016.7.20  重构代码，提高可读性/稳定性
* 2017.03.21 使用多线程接收
* 2017.04.10 使用HTTP/1.0协议，简化接收逻辑
* 2018.05.07 重构代码，添加对Header数据读写，设计更合理，灵活性更高
* 蒋晓岗<kerndev@foxmail.com>
* 2018.05.07
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kernel.h"
#include "log.h"
#include "modem.h"
#include "http.h"

#define MODULE_NAME     "HTTP"

#define MAX_HEAD_SIZE   (16 * 1024)
#define MAX_BODY_SIZE   (64 * 1024)

struct http_request
{
    char *head;
    char *body;
    int   head_size;
    int   body_size;
};

struct http_response
{
    int   code;
    char *head;
    char *body;
    int   head_size;
    int   body_size;
};

struct http_contex
{
    int socket;
    char head[MAX_HEAD_SIZE];
    char body[MAX_BODY_SIZE];
    thread_t thread;
    event_t event;
    struct http_request req;
    struct http_response resp;
};

//接收一行
static int http_recv_line(int socket, char *line, int size)
{
    int i;
    int ret;
    size = size - 1;    //留出1字节空间存放[\0]
    for(i = 0; i < size; i += ret)
    {
        ret = modem_recv(socket, line + i, 1);
		if(ret == 0)
        {
            continue;
        }
        if(ret < 0)
        {
            LOG("recv line error!\r\n");
            return 0;
        }
        if(line[i] == '\n')
        {
            i = i + 1;
            line[i] = 0;
            return i;
        }
    }
    LOG("recv line overflow!\r\n");
    return 0;
}

//接收响应状态
static int http_recv_code(struct http_contex *ctx)
{
    int ret;
    char *line;
    struct http_response *resp;
    resp = &ctx->resp;
    line = resp->head;
    resp->code = 0;
    ret = http_recv_line(ctx->socket, line, MAX_HEAD_SIZE);
    if(ret <= 2)
    {
        return 0;
    }
    if(strncmp(line, "HTTP/1.1", 8)==0)
    {
        LOG("%s", line);
        resp->code = atoi(line + 8);
        return 1;
    }
    return 0;
}

//接收响应头
static int http_recv_head(struct http_contex *ctx)
{
    int ret;
    int size;
    char *line;
    struct http_response *resp;
    resp = &ctx->resp;
    line = resp->head;
    size = MAX_HEAD_SIZE;
    resp->head_size = 0;
    while(size)
    {
        ret = http_recv_line(ctx->socket, line, size);
        if(ret == 0)
        {
            return 0;
        }
        if(ret == 2)
        {
            return 1;
        }
        line += ret;
        size -= ret;
        resp->head_size += ret;
    }
    LOG("recv head overflow!\r\n");
    return 0;
}

//接收响应正文
static int http_recv_body(struct http_contex *ctx)
{
    int ret;
    int size;
    char *body;
    struct http_response *resp;
    resp = &ctx->resp;
    body = resp->body;
    size = MAX_BODY_SIZE;
    resp->body_size = 0;
    while(size)
    {
        ret = modem_recv(ctx->socket, body, size);
        if(ret < 0)
        {
            *body = 0;
            return 1;
        }
        body += ret;
        size -= ret;
        resp->body_size += ret;
    }
    *body = 0;
    LOG("recv body overflow!\r\n");
    return 0;
}

//接收线程
static void http_thread_recv(void *arg)
{
    int ret;
    struct http_contex *ctx;
    ctx = (struct http_contex *)arg;
    
    //接收HEAD
    ret = http_recv_code(ctx);
    if(ret == 0)
    {
        LOG("recv code error!\r\n");
        event_post(ctx->event);
        return;
    }
    ret = http_recv_head(ctx);
    if(ret == 0)
    {
        LOG("recv head error!\r\n");
        event_post(ctx->event);
        return;
    }
    LOG("recv head ok, size=%d\r\n", ctx->resp.head_size);
     
    //接收BODY
    ret = http_recv_body(ctx);
    if(ret == 0)
    {
        LOG("recv body error!\r\n");
        event_post(ctx->event);
        return;
    }
    LOG("recv body ok, size=%d\r\n", ctx->resp.body_size);
    event_post(ctx->event);
    return;
}

static char *http_head_get_next_line(char *line)
{
    while(*line != '\n')
    {
        if(*line == 0x00)
        {
            return line;
        }
        line++;
    }
    return line + 1;
}

static int http_head_cmp_line_key(char *line, char *key)
{
    while(*key)
    {
        if(*line != *key)
        {
            return 0;
        }
        key++;
        line++;
    }
    return *line == ':';
}

static int http_head_get_line_value(char *line, char *value, int size)
{
    //查找分隔符
    while(*line != ':')
    {
        if(*line == '\r')
        {
            *value = 0;
            return 0;
        }
        line++;
    }
    line++;
    //跳过空白
    while(*line == ' ')
    {
        if(*line == '\r')
        {
            *value = 0;
            return 0;
        }
        line++;
    }
    //复制数据
    while(size > 0)
    {
        if(*line == '\r')
        {
            *value = 0;
            return 1;
        }
        *value = *line;
        value++;
        line++;
        size--;
    }
    *value = 0;
    LOG("get_line_value overflow!\r\n");
    return 1;
}

void http_request_init(http_t http, char *mothod, char *uri)
{
    struct http_contex *ctx;
    struct http_request *req;
    ctx = (struct http_contex *)http;
    req = &ctx->req;
    req->body_size = 0;
    req->head_size = sprintf(req->head, "%s %s HTTP/1.0\r\n", mothod, uri);
}

void http_request_set_head(http_t http, char *key, char *value)
{
    struct http_contex *ctx;
    struct http_request *req;
    ctx = (struct http_contex *)http;
    req = &ctx->req;
    req->head_size += sprintf(req->head + req->head_size, "%s: %s\r\n", key, value);
}

void http_request_set_body(http_t http, char *body, int size)
{
    struct http_contex *ctx;
    struct http_request *req;
    ctx = (struct http_contex *)http;
    req = &ctx->req;
    req->body = body;
    req->body_size = size;
    req->head_size += sprintf(req->head + req->head_size, "Content-Length: %d\r\n", size);
}

int http_response_get_code(http_t http)
{
    struct http_contex *ctx;
    struct http_response *resp;
    ctx = (struct http_contex *)http;
    resp = &ctx->resp;
    return resp->code;
}

int http_response_get_head(http_t http, char *key, char *value, int size)
{
    char *line;
    struct http_contex *ctx;
    struct http_response *resp;
    ctx = (struct http_contex *)http;
    resp = &ctx->resp;
    for(line = resp->head; *line != 0; line = http_head_get_next_line(line))
    {
        if(http_head_cmp_line_key(line, key))
        {
            return http_head_get_line_value(line, value, size);
        }
    }
    return 0;
}

int http_response_get_body(http_t http, char **body, int *size)
{
    struct http_contex *ctx;
    struct http_response *resp;
    ctx = (struct http_contex *)http;
    resp = &ctx->resp;
    *body = resp->body;
    *size = resp->body_size;
    return 1;
}

//HTTP发起请求
int http_request_send(http_t http, char *host, int port, int timeout)
{
    int ret;
    struct http_contex *ctx;
    struct http_request *req;
    struct http_response *resp;
    ctx = (struct http_contex *)http;
    req = &ctx->req;
    resp= &ctx->resp;
    resp->code = 0;
    
    //建立连接
    ret = modem_connect(ctx->socket, host, port);
    if(ret <= 0)
    {
        LOG("connect failed!\r\n");
        return 0;
    }
	http_request_set_head(http, "Host", host);
    http_request_set_head(http, "Connection", "close\r\n");
    
    //发送HTTP请求
    ret = modem_send(ctx->socket, req->head, req->head_size);
    if(ret <= 0)
    {
        LOG("send request head failed!\r\n");
		modem_close(ctx->socket);
        return 0;
    }
    if((req->body != NULL) && (req->body_size != 0))
    {
        ret=modem_send(ctx->socket, req->body, req->body_size);
        if(ret <= 0)
        {
            LOG("send request body failed!\r\n");
			modem_close(ctx->socket);
            return 0;
        }
    }
    LOG("send request ok.\r\n");
    
    //接收HTTP响应
    event_reset(ctx->event);
    ctx->thread = thread_create(http_thread_recv, ctx, 10240);
    ret = event_timed_wait(ctx->event, timeout);
    if(ret == 0)
    {
        LOG("recv response timeout!\r\n");
		modem_close(ctx->socket);
        event_wait(ctx->event);
        ctx->thread = NULL;
        return 0;
    }
	modem_close(ctx->socket);
    LOG("recv response %s\r\n", resp->code ? "ok." : "failed!");
    ctx->thread = NULL;
    return 1;
}

//创建一个http对象
http_t http_create(void)
{
    struct http_contex *ctx;
    struct http_request *req;
    struct http_response *resp;
    ctx = heap_alloc(sizeof(struct http_contex));
    if(ctx != NULL)
    {
        memset(ctx, 0, sizeof(struct http_contex));
        req = &ctx->req;
        req->head = ctx->head;
        req->body = ctx->body;
        resp = &ctx->resp;
        resp->head = ctx->head;
        resp->body = ctx->body;
        ctx->socket = modem_socket();
        ctx->event = event_create();
    }
    return ctx;
}

//删除一个http对象
void http_delete(http_t http)
{
    struct http_contex *ctx;
    ctx = (struct http_contex *)http;
    modem_delete(ctx->socket);
    event_delete(ctx->event);
    heap_free(ctx);
}
