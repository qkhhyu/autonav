#ifndef __SPI_H
#define __SPI_H

void spi_init(void);
void spi_open(int id, int baudrate);
void spi_close(int id);
void spi_send(int id, char data);
char spi_recv(int id);
void spi_send_bulk(int id, void *buff, int len);
void spi_recv_bulk(int id, void *buff, int len);

#endif
