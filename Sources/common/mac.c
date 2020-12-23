/******************************************************************************
* MAC算法
* MAC(Message Authentication Codes)带秘密密钥的Hash函数：
* 消息的散列值由只有通信双方知道的秘密密钥K来控制。
* 此时Hash值称作MAC。
******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "des.h"
#include "mac.h"

//按字节异或
static void memxor(uint8_t* dst, uint8_t* src, uint8_t* mask, int len)
{
	int i;
	for(i=0; i<len; i++)
	{
		dst[i] = src[i] ^ mask[i];
	}
} 

/******************************************************************************
* 基于DES加密的MAC算法
* 1. 将输入数据分割为8字节长的数据块组，标识为D1，D2，D3，D4...
*    余下的字节组成一个长度小于等于8字节的最后一块数据块.
* 2. 如果最后一个数据块长度等于8字节，则在此数据块后附加一个8字节长的数据块，
*    附加的数据块为：16进制"80 00 00 00 00 00 00 00".
*    如果最后一个数据块长度小于8字节，则该数据块的最后填补一个0x80.
*    如果填补之后数据块长度仍小于8字节，则在数据块后填补0x00至长度为8字节.
* 3. 将IV与D1进行异或,再用K1进行DES加密,得到密文C1.
* 4. C1与D2异或,再用K1进行DES加密,得到密文C2.
* 5. 依次类推,重复上述步骤,处理完所有数据块得到密文CX.
* 6. 将CX进行DES解密,密钥为K2,得到DCX.
* 7. 将DCX进行DES加密,密钥为K1,得到MAC
******************************************************************************/
void mac_des_cbc(uint8_t* out, uint8_t *in, int len, uint8_t *key, uint8_t* iv)
{
	int i;
	uint8_t buff[8];
	uint8_t cipher[8];
	
	//加密8字节的块
	memcpy(cipher, iv, 8);
	while(len >= 8)
	{
		memxor(buff, in, cipher, 8);
		des_encrypt(cipher, buff, key);
		in += 8;
		len-= 8;
	}
	
	//填充最后一块
	memset(buff, 0, 8);
	for(i=0; i<len; i++)
	{
		buff[i] = in[i];
	}
	buff[i] = 0x80;
	
	//加密最后一块
	memxor(buff, buff, cipher, 8);
	des_encrypt(cipher, buff, key);

	//生成MAC
	des_decrypt(buff, cipher, &key[8]);
	des_encrypt(out, buff, key);
}
