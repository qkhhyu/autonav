/*
* MPU9250 九轴传感器驱动
* MPU9250 = MPU6050 + AK8963
* MPU6050 I2C地址0xD0
* AK8963  I2C地址0x18
* 蒋晓岗<kerndev@foxmail.com>
* 2019.12.26
*/
#include "bsp.h"
#include "log.h"

#define I2C_PORT  2

//AK8963
static void ak8963_get_register(uint8_t addr, uint8_t *value)
{
	i2c_xfer_rx(I2C_PORT, 0x18, addr, value, 1);
}

static void ak8963_set_register(uint8_t addr, uint8_t value)
{
	i2c_xfer_tx(I2C_PORT, 0x18, addr, &value, 1);
}

//突发模式
static void ak8963_get_register_burst(uint8_t addr, uint8_t *value, uint8_t count)
{
	i2c_xfer_rx(I2C_PORT, 0x18, addr, value, count);
}

void ak8963_get_idcode(uint8_t *id)
{
	ak8963_get_register(0x00, id);
}

void ak8963_set_mode(uint8_t mode)
{
	ak8963_set_register(0x0A, 0x10 | mode);
}

void ak8963_get_st1(uint8_t *val)
{
	ak8963_get_register(0x02, val);
}

void ak8963_get_st2(uint8_t *val)
{
	ak8963_get_register(0x09, val);
}

void ak8963_get_asa(uint8_t coef[3])
{
	ak8963_get_register_burst(0x10, coef, 3);
}

void ak8963_get_mdm(int16_t axis[3])
{
	uint8_t *byte;
	byte = (uint8_t *)axis;
	ak8963_get_register_burst(0x03, byte, 6);
}

//-------------------------------------------------------------------------------------------------

//设置寄存器
static void mpu9250_set_register(uint8_t addr, uint8_t value)
{
	i2c_xfer_tx(I2C_PORT, 0xD0, addr, &value, 1);
}

//读取寄存器
static void mpu9250_get_register(uint8_t addr, uint8_t *value)
{
	i2c_xfer_rx(I2C_PORT, 0xD0, addr, value, 1);
}

//读取寄存器
static void mpu9250_get_register_burst(uint8_t addr, uint8_t *value, uint8_t count)
{
	i2c_xfer_rx(I2C_PORT, 0xD0, addr, value, count);
}

//通信成功，返回固定值0x71
void mpu9250_get_idcode(uint8_t *id)
{
	mpu9250_get_register(117, id);
}

void mpu9250_get_temp(int16_t *temp)
{
	uint8_t byte[2];
	int16_t raw;
	mpu9250_get_register(65, &byte[0]);
	mpu9250_get_register(66, &byte[1]);
	raw = (byte[0] << 8) | byte[1];
	*temp = (raw - 21) / 333.87f + 21;
}

void mpu9250_get_gyro(int16_t gyro[3])
{
	uint8_t byte[8];
	mpu9250_get_register_burst(67, byte, 6);
	gyro[0] = (byte[0] << 8) | byte[1];
	gyro[1] = (byte[2] << 8) | byte[3];
	gyro[2] = (byte[4] << 8) | byte[5];
}

void mpu9250_get_accel_offset(int16_t data[3])
{
	uint8_t byte[8];
	mpu9250_get_register_burst(119, byte, 6);
	data[0] = ((byte[0] << 8) | byte[1]);
	data[1] = ((byte[2] << 8) | byte[3]);
	data[2] = ((byte[4] << 8) | byte[5]);
	data[0] = data[0] >> 1;
	data[1] = data[1] >> 1;
	data[2] = data[2] >> 1;
}

void mpu9250_get_accel(int16_t acc[3])
{
	uint8_t byte[8];
	mpu9250_get_register_burst(59, byte, 6);
	acc[0] = (byte[0] << 8) | byte[1];
	acc[1] = (byte[2] << 8) | byte[3];
	acc[2] = (byte[4] << 8) | byte[5];
}

//读取加速度
int mpu9250_get_accel_fifo(int16_t acc[][3], uint16_t count)
{
	uint8_t byte[8];
	uint16_t i;
	uint16_t fifo_cnt;
	mpu9250_get_register(58, &byte[0]);
	if(byte[0] & 0x10)
	{
		LOG("fifo over flow!\r\n");
		mpu9250_set_register(106, (1<<6)|(1<<2));  //清空FIFO
	}
	mpu9250_get_register(114, &byte[0]);
	mpu9250_get_register(115, &byte[1]);
	fifo_cnt = (byte[0] << 8) | byte[1];
	fifo_cnt = fifo_cnt / 6;
	count = (count < fifo_cnt) ? count : fifo_cnt;
	for(i=0; i<count; i++)
	{
		mpu9250_get_register(116, &byte[0]);
		mpu9250_get_register(116, &byte[1]);
		mpu9250_get_register(116, &byte[2]);
		mpu9250_get_register(116, &byte[3]);
		mpu9250_get_register(116, &byte[4]);
		mpu9250_get_register(116, &byte[5]);
		acc[i][0] = (byte[0] << 8) | byte[1];
		acc[i][1] = (byte[2] << 8) | byte[3];
		acc[i][2] = (byte[4] << 8) | byte[5];
	}
	return count;
}

//基本配置
void mpu9250_setup(void)
{
	//mpu9250_set_register(107, 1<<7);   //复位设置
	mpu9250_set_register(106, (1<<6)|(1<<2));  //使能FIFO
	mpu9250_set_register(25, 49);     //设置采样率1000/(1+49)=20Hz
	mpu9250_set_register(26, 0x04);   //设置低通滤波器
	mpu9250_set_register(27, 0x00);   //陀罗仪配置
	mpu9250_set_register(28, 0x00);   //加速度配置
	mpu9250_set_register(29, 0x06);   //设置加速度LPF
	mpu9250_set_register(30, 0x08);   //设置加速度ODR
	mpu9250_set_register(35, 1<<3);   //开启加速度写入FIFO
	mpu9250_set_register(55, 0x02);   //IIC直通模式，访问AK8963
}

static void mpu9250_gpio_init(void)
{
	gpio_open(PI, 1, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(PI, 1, 1);
	
	gpio_open(PC, 11, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(PC, 11, 0);
	
	gpio_open(PD, 2, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(PD, 2, 1);
}

void mpu9250_power_on(void)
{
	gpio_write(PI, 1, 0);
}

void mpu9250_power_off(void)
{
	gpio_write(PI, 1, 1);
}

void mpu9250_init(void)
{
	mpu9250_gpio_init();
}
