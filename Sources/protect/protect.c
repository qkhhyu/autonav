/*
* 加密保护
* 蒋晓岗<kerndev@foxmail.com>
* 2016.8.26
*/
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "uuid.h"
#include "gt102.h"
#include "sha256.h"
#include "protect.h"
#include "app.h"
#include "binconv.h"

#define MODULE_NAME  "AUTH"

//使用设备序列号生成UID
//根据<产品编码文档>序列号为13位字符
//填充3个0,转换为8字节UID
static void make_uid(uint8_t output[8])
{
	char serial[32];
	strcpy(serial,syscfg.device_sn);
	strcat(serial,"000");
	serial[16] = 0;
	hex_to_bin(serial, output, 16);
}

//使用CPUID生成key
static void make_key(uint8_t output[8])
{
	uint8_t i;
	uint8_t uuid[12];
	uuid_read(uuid);
	for(i=0;i<8;i++)
	{
		output[i] = uuid[i];
	}
}

//生成8字节随机数
static void make_rand(uint8_t output[8])
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		output[i] = 1<<i;
	}
}

//软件计算SHA256
static void soft_auth(uint32_t sha[8], uint8_t rand[8], uint8_t data)
{
	uint8_t mbox[64];
	memset(mbox,data,64);
	make_key(mbox);
	make_uid(mbox+40);
	memcpy(mbox+48, rand, 8);
	sha256(sha,mbox,64);
	//LOG("SOFT SHA256=%08X%08X%08X%08X%08X%08X%08X%08X\r\n",sha[0],sha[1],sha[2],sha[3],sha[4],sha[5],sha[6],sha[7]);
}

//初始化GT102
//Page0=0x00/0xAA
int protect_bind(void)
{
	int ret;
	uint8_t buff[32];
	//UID
	make_uid(buff);
	ret = gt102_set_uid(buff);
	if(!ret)
	{
		LOG("failed init uid!\r\n");
		return 0;
	}
	//KEY
	make_key(buff);
	ret = gt102_set_key(buff);
	if(!ret)
	{
		LOG("failed init key!\r\n");
		return 0;
	}
	LOG("bind ok.\r\n");
	return 1;
}

//GT102认证
int protect_auth(void)
{
	int ret;
	uint8_t  rand[8];
	uint32_t sha1[8];
	uint32_t sha2[8];
	
	//GT102计算SHA
	make_rand(rand);
	ret = gt102_auth_device(0, rand, (uint8_t*)sha1);
	if(!ret)
	{
		LOG("failed read auth buff!\r\n");
		return 0;
	}
	//LOG("CHIP SHA256=%08X%08X%08X%08X%08X%08X%08X%08X\r\n",sha1[0],sha1[1],sha1[2],sha1[3],sha1[4],sha1[5],sha1[6],sha1[7]);
	
	//软件计算SHA
	soft_auth(sha2, rand, 0x00);
	if(memcmp(sha1,sha2,32)==0)
	{
		LOG("chip data: 0x00\r\n");
		return 1;
	}
	soft_auth(sha2, rand, 0xAA);
	if(memcmp(sha1,sha2,32)==0)
	{
		LOG("chip data: 0xAA\r\n");
		return 1;
	}
	return 0;
}

void protect_init(void)
{
	uint32_t ver;
	gt102_init();
	gt102_version(&ver);
	LOG("chip rev.: 0x%08X\r\n",ver);	
}
