#ifndef __APPCFG_H
#define __APPCFG_H

//ҵ�����ò���
struct appcfg
{
	uint32_t magic;
	uint32_t crc32;
	
	char     device_id[32];				//�豸ID
	char     modem[32];					//������ʽ:ethernet/gprs/lte
	
	char     apn_name[32];				//APN�����:cmwap/cmnet/uninet...
	char     apn_user[32];				//APN�û���
	char     apn_pass[32];				//APN����
	
	char     bizp_addr[64];				//ҵ��ƽ̨��ַ
	int      bizp_port;					//ҵ��ƽ̨�˿�
	char     bizp_user[32];             //ҵ��ƽ̨��½�ʺ�
	char     bizp_pass[32];             //ҵ��ƽ̨��½����
	
	int      compass_offset;            //��������0��ƫ��
	int      compass_cal[3];            //��������У׼ֵ
};

void appcfg_read(struct appcfg *cfg);
void appcfg_write(struct appcfg *cfg);
void appcfg_reset(struct appcfg *cfg);

#endif
