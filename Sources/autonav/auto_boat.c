#include <string.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "sail.h"
#include "feed.h"

#include "auto_boat_cruise.h"

mutex_t auto_boat_mutex;


static void boat_left(int16_t speed1, int16_t speed2, int16_t dir)
{
	speed1 = speed2-(dir*10);
	LOG_HMI("left: speed=%d, %d, dir=%d\r\n", speed1, speed2, dir);

	


	sail_set_speed(speed1, speed2, -dir);
}

static void boat_right(int16_t speed1, int16_t speed2, int16_t dir)
{
	speed2 = speed1-(dir*10);
	LOG_HMI("right: speed=%d, %d, dir=%d\r\n", speed1, speed2, dir);
	sail_set_speed(speed1, speed2, dir);
}

static void boat_straight(int16_t speed1,int16_t speed2)
{
	LOG_HMI("straight: speed=%d, %d\r\n", speed1, speed2);
	sail_set_speed(speed1, speed2, 0);
}

static void boat_control(int16_t speed1, int16_t speed2, int16_t dir)
{
	if(dir<=0)
	{
		speed1 = speed2+(dir*10);
		speed1 = (speed1<(-1000))?-1000:speed1;
	}
	if(dir>0)
	{
		speed2 = speed1-(dir*10);
		speed2 = (speed2<(-1000))?-1000:speed2;
	}
	LOG_HMI("control: speed=%d, %d, dir=%d\r\n", speed1, speed2, dir);
	sail_set_speed(speed1, speed2, dir);
}

static void boat_feed(int16_t speed1, int16_t speed2)
{
	feed_set_speed(speed1);
	LOG_HMI("feed: speed1=%d, speed2=%d\r\n", speed1, speed2);
}

void auto_boat_init(void)
{
	struct cruise_handler handler;
	handler.turn_left = boat_left;
	handler.turn_right = boat_right;
	handler.go_strgaight = boat_straight;
	handler.control = boat_control;
	handler.feed = boat_feed;
	cruise_bind(&handler);
	auto_boat_mutex = mutex_create();
}

