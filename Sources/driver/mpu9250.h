#ifndef __MPU9250_H
#define __MPU9250_H

void mpu9250_init(void);
void mpu9250_power_on(void);
void mpu9250_power_off(void);
void mpu9250_setup(void);

//��ȡID��0x71
void mpu9250_get_idcode(uint8_t *id);

//��ȡ�¶�
void mpu9250_get_temp(int16_t *temp);

//��ȡ���ٶ�
void mpu9250_get_gyro(int16_t data[3]);

//��ȡ���ٶ�
void mpu9250_get_accel_offset(int16_t data[3]);
void mpu9250_get_accel(int16_t data[3]);
int  mpu9250_get_accel_fifo(int16_t data[][3], uint16_t count);

//-------------------------------------------------------------------------------------------------
//��ȡID��0x48
void ak8963_get_idcode(uint8_t *id);

//���ù���ģʽ
#define AK8963_MODE_IDLE   0x00
#define AK8963_MODE_SINGLE 0x01
#define AK8963_MODE_AUTO1  0x02
#define AK8963_MODE_AUTO2  0x06
#define AK8963_MODE_FROM   0x0F
void ak8963_set_mode(uint8_t mode);

//��ȡ״̬
#define AK8963_ST1_DRDY    0x01
#define AK8963_ST2_HOFL    0x08
void ak8963_get_st1(uint8_t *val);
void ak8963_get_st2(uint8_t *val);

//��ȡ����У׼ϵ��
void ak8963_get_asa(uint8_t coef[3]);

//��ȡ��������
void ak8963_get_mdm(int16_t data[3]);

#endif
