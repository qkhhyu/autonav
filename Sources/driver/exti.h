#ifndef __EXTI_H
#define __EXTI_H

#define EXTI_MODE_RISING	0x08
#define EXTI_MODE_FALLING	0x0C
#define EXTI_MODE_ALL		0x10  

void exti_init(void);
void exti_open(int port, int pin, int mode, void(*handler)(void));
void exti_close(int port, int pin);

#endif
