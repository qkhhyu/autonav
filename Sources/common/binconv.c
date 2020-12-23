/*
* 二进制转换工具
* 蒋晓岗<kerndev@foxmail.com>
* 2018.11.28
*/
#include <stdint.h>
#include "binconv.h"

static char hex_encode(uint8_t a)
{
	static const char table[]={"0123456789ABCDEF"};
    if(a < 16)
    {
        return table[a];
    }
    return table[0];
}

static uint8_t hex_decode(char ch)
{
    if((ch >= 'a') && (ch <= 'f'))
    {
        return (ch - 'a' + 10);
    }
    if((ch >= 'A') && (ch <= 'F'))
    {
        return (ch - 'A' + 10);
    }
    if((ch >= '0') && (ch <= '9'))
    {
        return (ch - '0');
    }
    return 0;
}

static uint8_t hex_to_byte(char *hex)
{
    uint8_t h1,h2;
    h1 = hex_decode(hex[0]);
    h2 = hex_decode(hex[1]);
    return (h1 << 4) | h2;
}

static void byte_to_hex(uint8_t byte, char *hex)
{
    hex[0] = hex_encode(byte >> 4 & 0x0F);
    hex[1] = hex_encode(byte >> 0 & 0x0F);
}


//二进制转16进制字符串
void bin_to_hex(void *bin, char *hex, uint32_t len)
{
	char *p = bin;
    while(len--)
	{
        byte_to_hex(*p++, hex);
        hex += 2;
	}
	*hex = 0;
}

//16进制字符串转二进制
uint32_t hex_to_bin(char *hex, void *bin, uint32_t len)
{
	uint32_t i;
	uint8_t *p = bin;
	for(i = 0; (i < len) && (*hex); i += 2)
	{
        *p++ = hex_to_byte(hex);
        hex += 2;
	}
	return (uint32_t)p - (uint32_t)bin;
}

//16进制字符串转32位整数
uint32_t hex_to_u32(char *hex, uint32_t len)
{
    uint8_t temp[8];
    uint8_t i;
    uint8_t cnt;
    uint32_t num;
    num = 0;
    cnt = hex_to_bin(hex, temp, len);
    for(i=0; i<cnt; i++)
    {
        num <<= 8;
        num |= temp[i];
    }
    return num;
}

//2进制转BCD
uint8_t bin_to_bcd(uint8_t bin)
{
	uint8_t var;
	var = bin / 10;
	var <<= 4;
	var |= bin % 10;
	return var;
}

//BCD转2进制
uint8_t bcd_to_bin(uint8_t bcd)
{
	uint8_t var;
	var = bcd >> 4;
	var *= 10;
	var += bcd & 0x0F;
	return var;
}

//BCD转32位整数
uint32_t bcd_to_u32(uint8_t *bcd, uint8_t len)
{
    uint8_t i;
    uint32_t var;
    var = 0;
    for(i=0; i<len; i++)
    {
        var = var * 100;
        var = var + bcd_to_bin(bcd[i]);
    }
    return var;
}

//32位整数转BCD
void u32_to_bcd(uint32_t u32, uint8_t *bcd, uint8_t len)
{
    uint8_t i;
    for(i=0; i<len; i++)
    {
        bcd[len - i - 1] = u32 % 100;
        u32 = u32 / 100;
    }
}

//大小端转换
void byte_swap_order(void *input, void *output, int size)
{
	int i;
	uint8_t *p_i;
	uint8_t *p_o;
	p_i = input;
	p_o = output;
	for(i=0; i<size; i++)
	{
		p_o[size - i - 1] = p_i[i];
	}
}
