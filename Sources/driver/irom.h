#ifndef __IROM_H
#define __IROM_H

uint32_t irom_erase(uint32_t addr, uint32_t len);
uint32_t irom_write(uint32_t addr, void* data, uint32_t len);
uint32_t irom_read(uint32_t addr, void* data, uint32_t len);
void irom_boot(uint32_t addr);

#endif
