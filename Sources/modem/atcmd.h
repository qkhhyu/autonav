#ifndef __ATCMD_H
#define __ATCMD_H

void atcmd_init(void);
void atcmd_enter_mutex(void);
void atcmd_leave_mutex(void);
void atcmd_send_line(char *line);
void atcmd_send_data(void *data, int len);
int  atcmd_recv_line(char *line, int len);
void atcmd_recv_data(void *data, int len);
void atcmd_create_handler(const char *keyword, int (*handler)(const char *));
void atcmd_delete_handler(const char *keyword, int (*handler)(const char *));

#endif
