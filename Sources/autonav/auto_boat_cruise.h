#ifndef __AUTO_BOAT_CRUISE_H
#define __AUTO_BOAT_CRUISE_H

// #define program0
#define program1
#define program_pid
//#define program_pid_1

#define program_slow_down	//减速过完宏定义


#define BOAT_SPEED 2.0f//正常行驶速度
#define CURVE_SPEED 1.6f//转弯速度

extern double set_boat_speed;
extern int motor_speed;

struct cruise_handler
{
	void (*turn_left)(int16_t speed1, int16_t speed2, int16_t dir);
	void (*turn_right)(int16_t speed1, int16_t speed2, int16_t dir);
	void (*go_strgaight)(int16_t speed1, int16_t speed2);
	void (*control)(int16_t speed1, int16_t speed2, int16_t dir);
	void (*feed)(int16_t speed1, int16_t speed2);
};


void cruise_gps_control(long double dist,int azimuth,int heading);
void cruise_gps_control_pid(long double dist,int azimuth,int heading,int rudderangle);
void curise_general_control(int16_t speed1, int16_t speed2, int16_t dir);
void curise_gps_control_pid_1(long double dist,int azimuth,int heading,int rudderangle);
void cruise_feed_control(int16_t speed1,int16_t speed2);
int cruise_dist_control(struct dist *dist);
void cruise_bind(struct cruise_handler *handler);
void auto_cruise_pid_init(void);
void auto_cruise_init(void);

#endif
