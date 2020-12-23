#ifndef __APPCFG_H
#define __APPCFG_H

//业务配置参数
struct appcfg
{
	uint32_t magic;
	uint32_t crc32;
	
	char     device_id[32];				//设备ID
	char     modem[32];					//联网方式:ethernet/gprs/lte
	
	char     apn_name[32];				//APN接入点:cmwap/cmnet/uninet...
	char     apn_user[32];				//APN用户名
	char     apn_pass[32];				//APN密码
	
	char     bizp_addr[64];				//业务平台地址
	int      bizp_port;					//业务平台端口
	char     bizp_user[32];             //业务平台登陆帐号
	char     bizp_pass[32];             //业务平台登陆密码
	
	int      compass_offset;            //电子罗盘0点偏移
	int      compass_cal[3];            //电子罗盘校准值
};

void appcfg_read(struct appcfg *cfg);
void appcfg_write(struct appcfg *cfg);
void appcfg_reset(struct appcfg *cfg);

#endif
