/*
* 保存轨迹数据
* 蒋晓岗<kerndev@foxmail.com>
*/
#include <string.h>
#include "kernel.h"
#include "app.h"
#include "w25qxx.h"
#include "memmap.h"
#include "track.h"

#define TRACK_POINT_SIZE (sizeof(struct track))
#define MAX_TRACK_POINT  (TRACKS_SIZE/TRACK_POINT_SIZE)

static int m_point_count;

//全FF是无效数据
static int is_point_valid(uint8_t data[16])
{
	int i;
	for(i=0; i<TRACK_POINT_SIZE; i++)
	{
		if(data[i] != 0xFF)
		{
			return 1;
		}
	}
	return 0;
}

void track_init(void)
{
	int i;
	uint8_t buf[TRACK_POINT_SIZE];
	uint32_t addr;
	addr = TRACKS_ADDR;
	for(i=0; i<MAX_TRACK_POINT; i++)
	{
		w25qxx_read(addr, buf, TRACK_POINT_SIZE);
		addr += TRACK_POINT_SIZE;
		if(!is_point_valid(buf))
		{
			break;
		}
	}
	m_point_count = i;
}

int track_write(int flag, struct track *track)
{
	uint32_t addr;
	if(flag)
	{
		m_point_count = 0;
		w25qxx_erase(TRACKS_ADDR, TRACKS_SIZE);
	}
	if(m_point_count < MAX_TRACK_POINT)
	{
		addr = TRACKS_ADDR + TRACK_POINT_SIZE * m_point_count;
		w25qxx_write(addr, track, TRACK_POINT_SIZE);
		m_point_count++;
		return 1;
	}
	return 0;
}

int track_read(struct track *track, int count)
{
	int ret;
	ret = m_point_count < count ? m_point_count : count;
	w25qxx_read(TRACKS_ADDR, track, TRACK_POINT_SIZE * ret);
	return ret;	
}
