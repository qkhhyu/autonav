#ifndef __SYSCFG_H
#define __SYSCFG_H

//信息机只读配置, 出厂后不可修改
struct syscfg
{
	uint32_t magic;
	uint32_t crc32;
	uint8_t  device_mac[8];	//6位MAC地址
	char     device_sn[16];	//13位序列号
};

void syscfg_read(struct syscfg* cfg);
void syscfg_write(struct syscfg* cfg);

#endif
