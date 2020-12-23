#ifndef __BOARD_H
#define __BOARD_H

void board_init(void);
void board_power_on(void);
void board_power_off(void);
void board_reset(void);
uint8_t board_reset_reason(void);

#endif
