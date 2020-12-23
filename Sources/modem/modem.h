#ifndef __MODEM_H
#define __MODEM_H

int  modem_init(void);
int  modem_boot(void);
int  modem_reboot(void);
int  modem_rssi(void);
int  modem_socket(void);
void modem_delete(int sock);
int  modem_connect(int sock, const char *addr, int port);
void modem_close(int sock);
int  modem_send(int sock, void *data, int len);
int  modem_recv(int sock, void *data, int len);

#endif
