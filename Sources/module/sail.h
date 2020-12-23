#ifndef __SAIL_H
#define __SAIL_H

void sail_init(void);
int  sail_set_speed(int16_t speed1, int16_t speed2, int16_t angle);
int  sail_get_state(struct motor state[3]);

#endif
