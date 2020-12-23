/******************************************************************************
* MAC�㷨
* MAC(Message Authentication Codes)��������Կ��Hash������
* ��Ϣ��ɢ��ֵ��ֻ��ͨ��˫��֪����������ԿK�����ơ�
* ��ʱHashֵ����MAC��
******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "des.h"
#include "mac.h"

//���ֽ����
static void memxor(uint8_t* dst, uint8_t* src, uint8_t* mask, int len)
{
	int i;
	for(i=0; i<len; i++)
	{
		dst[i] = src[i] ^ mask[i];
	}
} 

/******************************************************************************
* ����DES���ܵ�MAC�㷨
* 1. ���������ݷָ�Ϊ8�ֽڳ������ݿ��飬��ʶΪD1��D2��D3��D4...
*    ���µ��ֽ����һ������С�ڵ���8�ֽڵ����һ�����ݿ�.
* 2. ������һ�����ݿ鳤�ȵ���8�ֽڣ����ڴ����ݿ�󸽼�һ��8�ֽڳ������ݿ飬
*    ���ӵ����ݿ�Ϊ��16����"80 00 00 00 00 00 00 00".
*    ������һ�����ݿ鳤��С��8�ֽڣ�������ݿ������һ��0x80.
*    ����֮�����ݿ鳤����С��8�ֽڣ��������ݿ���0x00������Ϊ8�ֽ�.
* 3. ��IV��D1�������,����K1����DES����,�õ�����C1.
* 4. C1��D2���,����K1����DES����,�õ�����C2.
* 5. ��������,�ظ���������,�������������ݿ�õ�����CX.
* 6. ��CX����DES����,��ԿΪK2,�õ�DCX.
* 7. ��DCX����DES����,��ԿΪK1,�õ�MAC
******************************************************************************/
void mac_des_cbc(uint8_t* out, uint8_t *in, int len, uint8_t *key, uint8_t* iv)
{
	int i;
	uint8_t buff[8];
	uint8_t cipher[8];
	
	//����8�ֽڵĿ�
	memcpy(cipher, iv, 8);
	while(len >= 8)
	{
		memxor(buff, in, cipher, 8);
		des_encrypt(cipher, buff, key);
		in += 8;
		len-= 8;
	}
	
	//������һ��
	memset(buff, 0, 8);
	for(i=0; i<len; i++)
	{
		buff[i] = in[i];
	}
	buff[i] = 0x80;
	
	//�������һ��
	memxor(buff, buff, cipher, 8);
	des_encrypt(cipher, buff, key);

	//����MAC
	des_decrypt(buff, cipher, &key[8]);
	des_encrypt(out, buff, key);
}
