#ifndef __CRC_H
#define __CRC_H

uint8_t crc8_calc(uint8_t init, void *data, uint32_t len);
uint16_t crc16_calc(uint16_t init, void *data, uint32_t len);
uint32_t crc32_calc(uint32_t init, void *data, uint32_t len);

#endif
