#ifndef __BINCONV_H
#define __BINCONV_H

//二进制转16进制字符串
void bin_to_hex(void *bin, char *hex, uint32_t len);

//16进制字符串转二进制
uint32_t hex_to_bin(char *hex, void *bin, uint32_t len);

//16进制字符串转32位整数
uint32_t hex_to_u32(char *hex, uint32_t len);

//2进制转BCD
uint8_t bin_to_bcd(uint8_t bin);

//BCD转2进制
uint8_t bcd_to_bin(uint8_t bcd);

//BCD转32位整数
uint32_t bcd_to_u32(uint8_t *bcd, uint8_t len);

//32位整数转BCD
void u32_to_bcd(uint32_t u32, uint8_t *bcd, uint8_t len);

//大小端转换
void byte_swap_order(void *input, void *output, int size);

#endif
