/**************************************************************************************************
* 通用串行数据输出设备
* 蒋晓岗<kerndev@foxmail.com>
* 2019.05.06
**************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include "uart.h"
#include "serial.h"

#define UART_PORT		1
#define UART_BAUDRATE	921600
#define BUFF_SIZE       10240

static char m_string[BUFF_SIZE];

static void mem_dump_8bit(const uint8_t *p, int count)
{
    int len;
    while(count--)
    {
        len = sprintf(m_string, "%02X ", *p++);
        uart_write(UART_PORT, m_string, len);
    }
}

static void mem_dump_16bit(const uint16_t *p, int count)
{
    int len;
    while(count--)
    {
        len = sprintf(m_string, "%04X ", *p++);
        uart_write(UART_PORT, m_string, len);
    }
}

static void mem_dump_32bit(const uint32_t *p, int count)
{
    int len;
    while(count--)
    {
        len = sprintf(m_string, "%08X ", *p++);
        uart_write(UART_PORT, m_string, len);
    }
}

void serial_dump(const void *p, int count, int bits)
{
    if(bits == 8)
    {
        mem_dump_8bit((const uint8_t*)p, count);
        return;
    }
    if(bits == 16)
    {
        mem_dump_16bit((const uint16_t*)p, count);
        return;
    }
    if(bits == 32)
    {
        mem_dump_32bit((const uint32_t*)p, count);
        return;
    }
}

void serial_puts(const char *s)
{
    while(*s)
    {
        uart_write(UART_PORT, (void *)s++, 1);
    }
}

void serial_printf(const char *fmt, ...)
{
    int len;
	va_list va;
	va_start(va, fmt);
	len = vsnprintf(m_string, BUFF_SIZE, fmt, va);
	va_end(va);
	uart_write(UART_PORT, m_string, len);
}

void serial_init(void)
{
    uart_init(UART_PORT, NULL, 0);
	uart_open(UART_PORT, UART_BAUDRATE, 0);
}
