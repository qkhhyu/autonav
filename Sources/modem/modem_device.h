#ifndef __MODEM_DEVICE_H
#define __MODEM_DEVICE_H

struct modem_device
{
	int (*boot)(void);
	int (*rssi)(void);
	int (*socket_create)(void);
	void(*socket_delete)(int);
	int (*socket_connect)(int, const char *, int);
	void(*socket_close)(int);
	int (*socket_send)(int, void *, int);
	int (*socket_recv)(int, void *, int);
};

void modem_device_bind(struct modem_device *device);

#endif
