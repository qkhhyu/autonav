#ifndef __FEED_H
#define __FEED_H

void feed_init(void);
int  feed_set_speed(int16_t speed1, int16_t speed2);
int  feed_get_state(struct motor state[2]);

#endif
