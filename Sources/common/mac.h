#ifndef __MAC_H
#define __MAC_H

/******************************************************************************
* DES-CBC模式MAC(Message Authentication Codes)算法
* 输入任意长度的数据，输出8字节的MAC，用于数据校验
* out: [输出]8字节MAC
* key: [输入]16字节密钥
* iv : [输入]8字节初始值
* in : [输入]输入数据
* len: [输入]输入数据的长度
*******************************************************************************/
void mac_des_cbc(uint8_t* out, uint8_t *in, int len, uint8_t *key, uint8_t* iv);

#endif
