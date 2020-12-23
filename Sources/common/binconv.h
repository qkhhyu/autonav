#ifndef __BINCONV_H
#define __BINCONV_H

//������ת16�����ַ���
void bin_to_hex(void *bin, char *hex, uint32_t len);

//16�����ַ���ת������
uint32_t hex_to_bin(char *hex, void *bin, uint32_t len);

//16�����ַ���ת32λ����
uint32_t hex_to_u32(char *hex, uint32_t len);

//2����תBCD
uint8_t bin_to_bcd(uint8_t bin);

//BCDת2����
uint8_t bcd_to_bin(uint8_t bcd);

//BCDת32λ����
uint32_t bcd_to_u32(uint8_t *bcd, uint8_t len);

//32λ����תBCD
void u32_to_bcd(uint32_t u32, uint8_t *bcd, uint8_t len);

//��С��ת��
void byte_swap_order(void *input, void *output, int size);

#endif
