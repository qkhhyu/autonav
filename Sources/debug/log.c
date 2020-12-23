#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "kernel.h"
#include "define.h"
#include "serial.h"
#include "bsp.h"
#include "log.h"

#define MAX_BUFF_LEN    (64 * 1024)

static mutex_t m_mutex;
static char *  m_string;

void log_dump(const char *str, void *mem, int count, int bits)
{
    mutex_lock(m_mutex);
	serial_puts(str);
    serial_dump(mem, count, bits);
    serial_puts("\r\n");
    mutex_unlock(m_mutex);
}

void log_puts(bool show_time, const char *fmt, ...)
{
	va_list va;
	struct date date;
	mutex_lock(m_mutex);
	if(show_time)
    {
		time_get_date(&date);
		sprintf(m_string, "[%02d:%02d:%02d]", date.hour, date.minute, date.second);
		serial_puts(m_string);
	}
	va_start(va, fmt);
	vsnprintf(m_string, MAX_BUFF_LEN, fmt, va);
	va_end(va);
    serial_puts(m_string);
	mutex_unlock(m_mutex);
}

void log_init(void)
{
	m_mutex = mutex_create();
    m_string = heap_alloc(MAX_BUFF_LEN);
    serial_init();
}
