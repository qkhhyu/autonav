/*
* LM75A温度传感器驱动
* 蒋晓岗<kerndev@foxmail.com>
* 2019.03.12
*/
#include <stdint.h>
#include <stdbool.h>
#include "bsp.h"
#include "log.h"

#define MODULE_NAME "LM75"

#define I2C_ADDR    (0x90)
#define I2C_PORT    1

static int lm75_read_reg16(uint8_t addr, uint8_t *data)
{
	int ret;
    ret = i2c_xfer_rx(I2C_PORT, I2C_ADDR, addr, data, 1);
    return ret;
}

int lm75_read(void)
{
    uint8_t data[2];
    if(lm75_read_reg16(0, data) == 0)
    {
        LOG("read temp failed!\r\n");
        return 0;
    }
    //LOG(": %02X %02X\r\n", data[0], data[1]);
    return (int)data[0];
}

void lm75_init(void)
{
	gpio_open(PH, 15, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(PH, 15, 0);
}
