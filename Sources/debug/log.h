#ifndef __LOG_H
#define __LOG_H

void log_init(void);
void log_dump(const char *str, void *mem, int count, int bits);
void log_puts(bool show_time, const char *fmt, ...);

#define DBG(fmt...)             log_puts(false, fmt)
#define LOG(fmt...)             log_puts(true, fmt)
#define DUMP(str, mem, size)    log_dump(str, mem, size, 8)
#define LOG_INFO                LOG
#define LOG_DUMP                DUMP

#endif
