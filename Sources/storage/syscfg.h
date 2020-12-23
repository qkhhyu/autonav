#ifndef __SYSCFG_H
#define __SYSCFG_H

//��Ϣ��ֻ������, �����󲻿��޸�
struct syscfg
{
	uint32_t magic;
	uint32_t crc32;
	uint8_t  device_mac[8];	//6λMAC��ַ
	char     device_sn[16];	//13λ���к�
};

void syscfg_read(struct syscfg* cfg);
void syscfg_write(struct syscfg* cfg);

#endif
