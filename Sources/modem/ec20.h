#ifndef __EC20_H
#define __EC20_H

void         ec20_init(void);
void         ec20_start(void);
int	         ec20_get_rssi(void);
const char * ec20_get_name(void);
const char * ec20_get_addr(void);
int          ec20_socket_create(void);
void         ec20_socket_delete(int sock);
int          ec20_socket_connect(int sock, const char *addr, int port);
void         ec20_socket_close(int sock);
int          ec20_socket_send(int sock, void *data, int len);
int          ec20_socket_recv(int sock, void *data, int len);

#endif
