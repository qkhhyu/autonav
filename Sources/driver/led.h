#ifndef __LED_H
#define __LED_H

#define LED_RED		0
#define LED_GREEN	1

void led_init(void);
void led_open(int id);
void led_close(int id);
void led_write(int id, int data);

#endif
