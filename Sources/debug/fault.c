/**************************************************************************************************
* ³ÌÐò´íÎó²¶×½
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2019.05.06
**************************************************************************************************/
#include "stm32f4xx.h"
#include "kernel.h"
#include "serial.h"

void HardFault_Handler(void)
{
    register uint32_t sp;
    sp = __get_MSP();
    serial_puts("\r\nstack dump:\r\n");
    serial_dump((void *)sp, 16, 32);
    serial_puts("\r\nthread dump:\r\n");
    serial_dump(thread_self(), 16, 32);
    serial_puts("\r\nsystem fault!\r\n");
    while(1);
}
