#ifndef __W25QXXX_H
#define __W25QXXX_H

void w25qxx_init(void);
uint32_t w25qxx_erase(uint32_t addr, uint32_t len);
uint32_t w25qxx_read(uint32_t addr, void* buf, uint32_t len);
uint32_t w25qxx_write(uint32_t addr, void* buf, uint32_t len);

#endif
