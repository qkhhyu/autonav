#ifndef __SERIAL_H
#define __SERIAL_H

void serial_init(void);
void serial_dump(const void *p, int count, int bits);
void serial_puts(const char *s);
void serial_printf(const char *fmt, ...);

#endif
