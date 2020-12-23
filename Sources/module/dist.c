/*
* ³¬Éù²¨²â¾àÄ£¿é
* ½¯Ïþ¸Ú<kerndev@foxmail.com>
* 2019.12.28
*/

#include "kernel.h"
#include "bsp.h"
#include "log.h"

#define UART_PORT	2
#define UART_BAUD   115200

static mutex_t  m_mutex;
static event_t  m_event;
static uint16_t m_dist;

//¼ÆËã¼ìÑéºÍ
static uint8_t calc_checksum(uint8_t *data, int size)
{
	uint8_t sum;
	int i;
	sum = 0;
	for(i=0; i<size; i++)
	{
		sum = sum + data[i];
	}
	return sum;
}

//·¢ËÍÃüÁî
static void disp_send_command(uint8_t addr, uint8_t cmd, uint8_t data[4])
{
	uint8_t frame[16];
	frame[0] = 0x7F;
	frame[1] = addr;
	frame[2] = cmd;
	frame[3] = 0x00;
	frame[4] = data[0];
	frame[5] = data[1];
	frame[6] = data[2];
	frame[7] = data[3];
	frame[8] = 0x03;
	frame[9] = calc_checksum(&frame[1], 8);
	uart_write(UART_PORT, frame, 10);
}

//´¦ÀíÊý¾Ý
static void dist_proc_command(uint8_t addr, uint8_t cmd, uint8_t data[4])
{
	uint16_t dist;
	if(cmd == 0x12)
	{
		dist = (data[0] << 8) | data[1];
		//LOG("addr=%d, dist=%umm\r\n", addr, dist);
		m_dist = dist;
		event_post(m_event);
	}
}

//½âÎöÃüÁî
static void dist_recv_command(uint8_t *stream, int size)
{
	static uint8_t frame[16];
	static uint8_t frame_size;
	int i;
	for(i=0; i<size; i++)
	{
		frame[frame_size++] = stream[i];
		if(frame_size == 1)
		{
			if(stream[i] != 0x7F)
			{
				frame_size = 0;
			}
			continue;
		}
		if(frame_size == 9)
		{
			frame_size = 0;
			dist_proc_command(frame[1], frame[2], &frame[4]);
			//LOG_DUMP(frame, 10);
		}
	}
}

static void dist_thread(void *arg)
{
	uint8_t buf[260];
	int ret;
	while(1)
	{
		ret = uart_read(UART_PORT, buf, 256);
		if(ret > 0)
		{
			dist_recv_command(buf, ret);
		}
		else
		{
			sleep(10);
		}
	}
}

bool dist_read(int id, uint16_t *dist)
{
	bool ret;
	uint8_t data[4];
	mutex_lock(m_mutex);
	event_reset(m_event);
	m_dist = 0;
	disp_send_command(id, 0x12, data);
	ret = event_timed_wait(m_event, 50);
	*dist = m_dist;
	mutex_unlock(m_mutex);
	return ret;
}

void dist_init(void)
{
	m_mutex = mutex_create();
	m_event = event_create();
	gpio_open(PD, 3, GPIO_MODE_OUT, GPIO_OUT_PP);
	gpio_write(PD, 3, 0);
	uart_init(UART_PORT, heap_alloc(1024), 1024);
	uart_open(UART_PORT, UART_BAUD, 0);
	thread_create(dist_thread, 0, 10240);
}
