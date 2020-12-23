/*
* MPU9250驱动服务
* 蒋晓岗<kerndev@foxmail.com>
* 2020.09.09
*/
#include <math.h>
#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "svc.h"
#include "hmi.h"

static float m_mdm_coef[3];

//-------------------------------------------------------------------------------------------------
//等待MPU9250通信成功
static void init_hardware(void)
{
	uint8_t id;
	uint8_t data[3];
	
	mpu9250_power_off();
	sleep(100);
	mpu9250_power_on();
	
	for(id = 0; id != 0x71; )
	{
		sleep(1000);
		mpu9250_get_idcode(&id);
		LOG("mpu6050 id=0x%02X\r\n", id);
	}
	mpu9250_setup();
	for(id = 0; id != 0x48; )
	{
		sleep(1000);
		ak8963_get_idcode(&id);
		LOG("ak8963 cid=0x%02X\r\n", id);
	}
	
	ak8963_set_mode(AK8963_MODE_FROM);
	sleep(10);
	ak8963_get_asa(data);
	ak8963_set_mode(AK8963_MODE_AUTO2);
	m_mdm_coef[0] = 1 + ((data[0] - 128) * 0.5f / 128);
	m_mdm_coef[1] = 1 + ((data[1] - 128) * 0.5f / 128);
	m_mdm_coef[2] = 1 + ((data[2] - 128) * 0.5f / 128);
	LOG("ak8963 coef: %f, %f, %f\r\n", m_mdm_coef[0], m_mdm_coef[1], m_mdm_coef[2]);
}

//采样数据
static void sample_data(void)
{
	int16_t acc[3];
	int16_t mdm[3];
	uint8_t stx[2];
	
	mpu9250_get_accel(acc);
	//LOG("A:X=%d,Y=%d,Z=%d\r\n", acc[0],acc[1],acc[2]);
	svc_hittest_input(acc);
	
	ak8963_get_mdm(mdm);
	ak8963_get_st2(stx);
	mdm[0] = mdm[0] * m_mdm_coef[0];
	mdm[1] = mdm[1] * m_mdm_coef[1];
	mdm[2] = mdm[2] * m_mdm_coef[2];
	//LOG_HMI("data: %d, %d, %d\r\n", mdm[0], mdm[1], mdm[2]);
	svc_compass_input(mdm, acc);
}

//工作线程
static void svc_mpu9250_thread(void *arg)
{
	LOG("[SVC:9250]init.\r\n");
	init_hardware();
	while(1)
	{
		sample_data();
		thread_sleep(50);
	}
}

void svc_mpu9250_init(void)
{
	svc_hittest_init();
	svc_compass_init();
	thread_create(svc_mpu9250_thread, 0, 10240);
}
