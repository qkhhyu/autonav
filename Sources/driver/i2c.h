#ifndef __I2C_H
#define __I2C_H

void i2c_init(void);
int i2c_xfer_rx(int id, uint8_t addr, uint8_t reg, uint8_t *buf, int size);
int i2c_xfer_tx(int id, uint8_t addr, uint8_t reg, uint8_t *buf, int size);

#endif
