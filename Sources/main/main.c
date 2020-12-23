/*
* ³ÌÐòÈë¿Ú(0x08040000)
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
*/
#include "kernel.h"
#include "board.h"
#include "sdram.h"
#include "app.h"
#include "bsp.h"
#include "log.h"

void init(void *arg)
{
	sleep(1000);
    log_init();
	bsp_init();
	app_init();
}

void idle(void *arg)
{
    kernel_idle();
}

int main(void)
{
	board_init();
	sdram_init();
	kernel_init(SDRAM_ADDR,SDRAM_SIZE);
	thread_create(init, NULL, 0);
    thread_create(idle, NULL, 0);
	kernel_start();
}
