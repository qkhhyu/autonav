#include <string.h>
#include <math.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "sail.h"
#include "gnss.h"
#include "svc_gnss.h"
#include "svc_compass.h"

#include "auto_boat.h"
#include "auto_location.h"
#include "auto_boat_cruise.h"
#include "auto_dist.h"
#include "auto_pid.h"

#define MAX_TRACKS	1000

#define PI (3.1415926535898)
#define R_earth (6371393)

#define radian2angle(radian)	(180.0/PI*(radian))
#define angle2radian(angle)		(PI/180*(angle))

struct location_contex
{
	uint8_t feed_switch;//喂料状态 0关闭，1等待开启，2开启
	uint8_t auto_sail;//巡航状态
};

struct location_contex location_contex;




thread_t location_thread_t = NULL;
event_t  location_event;
mutex_t location_mutex;
int location_thread_switch = 0;

long double destlatitude = 0;
long double destlongitude = 0;
long double currentlatitude = 0;
long double currentlongitude = 0;
static double currentspeed = 0;
int currentheading;
static struct dist currentdist;

struct track tracks[MAX_TRACKS];



//设定目标点经纬度坐标
void auto_location_set_destination(long double latitude,long double longitude)
{
	mutex_lock(location_mutex);
	destlatitude = latitude;
	destlongitude = longitude;
	mutex_unlock(location_mutex);	
	LOG("[AUTO:LOCATION]set destination destlatitude=%lf destlongitude=%lf\r\n",destlatitude,destlongitude);
}

extern void auto_get_gpsspeed(struct gnss *gnss);
//获取设置当前位置经纬度坐标
static void auto_get_gpslocation(struct gnss *gnss)
{
	mutex_lock(location_mutex);
	currentlatitude = gnss->latitude;
	currentlongitude = gnss->longitude;
	currentspeed = gnss->speed;
	mutex_unlock(location_mutex);	
	LOG("[SVC:GPS]%s,%+.9lf,%+.9lf,speed=%f, course=%f\r\n", gnss->locked ? "OK":"NA",  gnss->latitude, gnss->longitude, gnss->speed, gnss->course);
	LOG("[SVC:COMP],heading=%d,\r\n", currentheading);
	auto_get_gpsspeed(gnss);
	event_post(location_event);
}

//获取超声波距离信息
static void auto_get_dist(struct dist *dist)
{
	memcpy(&currentdist,dist,sizeof(struct dist));
}

//计算两个坐标之间的距离和方位角
static void auto_algo_gps(long double start_latitude,long double start_longitude,long double via_latitude,long double via_longitude,long double *distance,int *azimuth)
{	
	long double angle_A = 0;
	long double L = 0;
	long double K_ab = 0;
	L = 2*R_earth*asin(sqrt( pow(sin(angle2radian((via_latitude-start_latitude)/2)),2) + cos(angle2radian(start_latitude)) * cos(angle2radian(via_latitude)) *pow(sin(angle2radian((via_longitude-start_longitude)/2)),2) ));
	// LOG_HMI("[AUTO:LOCATION],L = %lf,\r\n",L);
	*distance = L;

	//LOG("平面直角坐标系法\r\n");
	angle_A = radian2angle(atan( (via_longitude-start_longitude) * cos(angle2radian(via_latitude)) / (via_latitude-start_latitude)));
		//判断象限	
	if(via_longitude != start_longitude)
	{
		K_ab = (via_latitude - start_latitude) / (via_longitude - start_longitude);
		//LOG("K_ab = %lf\r\n", K_ab);
		if(K_ab>0)//一三象限
		{
			if(via_longitude>start_longitude)//第一象限
			{
				angle_A = angle_A;
			}
			else//第三象限
			{
				angle_A =180 + angle_A; 
			}	
		}
		else if (K_ab<0)//二四象限
		{
			if(via_longitude>start_longitude)//第四象限
			{
				angle_A =180 + angle_A;	
			}
			else
			{
				angle_A += 360.0;
			}	
		}
		else//k=0平行于X轴
		{
			if(via_longitude>start_longitude)
			{
				angle_A = angle_A;
			}
			else
			{
				angle_A += 360.0;
			}	
		}
	}
	else
	{
		if(via_latitude>start_latitude)
		{
			angle_A = angle_A;
		}
		else
		{
			angle_A =180 + angle_A;
		}
		
	}
	// LOG_HMI("[AUTO:LOCATION],A = %lf,\r\n",angle_A);
	// LOG_HMI("\r\n");
	*azimuth = (int)angle_A;
	
}

static void auto_location_get_heading(int heading)
{
	currentheading = heading;
	static int div;
	if(++div > 20)
	{
		div = 0;
		LOG_HMI("[SVC:COMP]heading=%d\r\n", heading);
	}
}

struct corner_contex
{
	int azimuth_last;
	int azimuth_next;
	uint8_t corner_action;
};

struct track_index
{
	int tracksnum;
	int current_track_index;
	int last_track_index;
	int next_track_index;
};

//获取最近的坐标点索引，和轨迹的总长度
//numoftrack：输出轨迹点总数
//track_index：最近轨迹点
//tdistance：总周长
static void auto_load_track(struct track_index *track_index,long double *tdistance)
{
	int i = 0;
	int azimuth = 0;
	long double distance =0;
	int tracksnum = 0;
	int current_track_index = 0;	
	long double distance_temp = 0; 
	double total_distance = 0;//总长度

	distance_temp = 0;
	current_track_index = 0;
	tracksnum = track_read(tracks,MAX_TRACKS);//获取所有轨迹点
	track_index->tracksnum = tracksnum;
	LOG("[AUTO:LOCATION]read tracksnum=%d\r\n",tracksnum);

	event_timed_wait(location_event, 1000);
	for(i=0;i<tracksnum;i++)//读取所有轨迹坐标，选择出与当前位置最近的一个坐标
	{
		//计算与当前位置的距离，用来查找与当前位置最近的一个坐标
		auto_algo_gps(currentlatitude,currentlongitude,tracks[i].latitude,tracks[i].longitude,&distance,&azimuth);
		if(i==0)
		{
			distance_temp = distance;
			current_track_index = 0;
			
		}
		if(distance>distance_temp)
		{
			current_track_index = current_track_index;
		}
		else//后续看看需不需要考虑相等的情况，按照当前方位角去找最近距离
		{
			distance_temp = distance;
			current_track_index = i;
		}
	}	
	current_track_index = (current_track_index+1)%tracksnum;//最近距离的下一个点，为自巡航的第一个点，避免船反向航行
	track_index->current_track_index = current_track_index;
	for(i=0;i<tracksnum;i++)
	{
		auto_algo_gps(tracks[i].latitude,tracks[i].longitude,tracks[(i+1)%tracksnum].latitude,tracks[(i+1)%tracksnum].longitude,&distance,&azimuth);
		//累加，计算轨迹的总距离，也就是虾塘的周长
		total_distance = total_distance+distance;		
	}
	*tdistance = total_distance;
	track_index->last_track_index = (current_track_index+tracksnum-1)%tracksnum;
	track_index->next_track_index = (current_track_index+1)%tracksnum;
	
}

//转弯判断
static void auto_calc_corner(struct track_index *track_index,struct corner_contex *corner_contex)
{
	int current_track_index = 0;
	int last_track_index = 0;
	int next_track_index = 0;
	long double dist_last;
	long double dist_next;
	int azimuth_last;
	int azimuth_next;
	int corner = 0;
	int pretreatment_judgment = 0;//转弯角度过小判断标记

	current_track_index = track_index->current_track_index;
	//计算上一个点跟当前目标点的距离和方位角
	last_track_index = 	track_index->last_track_index;		
	auto_algo_gps(tracks[last_track_index].latitude,tracks[last_track_index].longitude,tracks[current_track_index].latitude,tracks[current_track_index].longitude,&dist_last,&azimuth_last);
	//计算下一个点跟当前目标点的距离和方位角
	next_track_index = track_index->next_track_index;
	auto_algo_gps(tracks[current_track_index].latitude,tracks[current_track_index].longitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,&dist_next,&azimuth_next);

	//计算下一个点转弯的角度
	corner = ((360-azimuth_last)+azimuth_next)%360;
	if(corner<=180)
	{
		corner = corner;			
	}
	else
	{
		corner = 360-corner;
	}
	//转弯角度太小，也就是偏转角差值太大，需要考虑提前量以及减速处理
	if(corner>60)
	{
		pretreatment_judgment = 1;//角度过小，需要提前处理转弯点
	}
	else
	{
		pretreatment_judgment = 0;//安全运行转弯角度，不需要提前处理
	}
	
	if(pretreatment_judgment == 0)
	{
		//两个点之间距离太近，直接跳过，循迹下一个点
		if(dist_next<4.0f)
		{
			track_index->current_track_index++;
			if(track_index->current_track_index>=track_index->tracksnum)
			{
				track_index->current_track_index = 0;
			}
			//计算上一个点跟当前目标点的距离和方位角
			//这里上一个点不变
			auto_algo_gps(tracks[last_track_index].latitude,tracks[last_track_index].longitude,tracks[current_track_index].latitude,tracks[current_track_index].longitude,&dist_last,&azimuth_last);
			//计算下一个点跟当前目标点的距离和方位角
			next_track_index = (track_index->current_track_index+1)%track_index->tracksnum;
			auto_algo_gps(tracks[current_track_index].latitude,tracks[current_track_index].longitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,&dist_next,&azimuth_next);
			//计算下一个点转弯的角度
			corner = ((360-azimuth_last)+azimuth_next)%360;
			if(corner<=180)
			{
				corner = corner;			
			}
			else
			{
				corner = 360-corner;
			}
			//转弯角度太小，也就是偏转角差值太大，需要考虑提前量以及减速处理
			if(corner>60)
			{
				pretreatment_judgment = 1;//角度过小，需要提前处理转弯点
			}
			else
			{
				pretreatment_judgment = 0;//安全运行转弯角度，不需要提前处理
			}
		}
	}
	corner_contex->corner_action = pretreatment_judgment;
	corner_contex->azimuth_last = azimuth_last;
	corner_contex->azimuth_next = azimuth_next;
}

static int auto_change_track(uint8_t flag,long double distance,double lastdistance,int heading,struct corner_contex corner_contex)
{
	int azimuth_difference = 0;
	//判断距离是否开始远离目标点，远离则表示已经经过该点
	if(corner_contex.corner_action == 1)//要转大弯，提前结束该点循迹
	{
		if(distance<4.0f)
		{
			if((distance-lastdistance) > 0.07f)//远离目标点则认为已经经过该点
			{
				return 1;
			}
		}
		if(distance<2.0)
		{
			return 1;
		}
	}
	else//不需要转大弯，则不需要提前处理
	{
		if(distance<3.0f)
		{
			//这里可能需要再加一层判断，上次去厦门，这里速度太慢的时候一直不满足，就导致了打转的现象发生
			if((distance-lastdistance) > 0.07f)//远离目标点则认为已经经过该点
			{
				return 1;
			}
		}
//		if(flag != 1)
		{
			if(lastdistance<3.0f&&distance>3.0)//防止速度过慢零界点来回跳转，会造成打转的现象！
			{
				return 1;
			}
		}
		//距离小于1米时也认为已经到达目标点
		if(distance <1.0)
		{
			return 1;
		}
		if(distance<6.0f)
		{
			//巡航途中，如果有反向航行趋势，要即时调整，避免反向航线
			if(location_contex.auto_sail ==2)
			{
				azimuth_difference = ((360-heading)+corner_contex.azimuth_last)%360;
				if(azimuth_difference<=180)
				{
					azimuth_difference = azimuth_difference;
				}
				else
				{
					azimuth_difference = 360-azimuth_difference;
				}
				if(azimuth_difference>=90)//船身如果过了该点正切线（还是法线？）则认为已经过了该点，防止打转情况发生
				{
					return 1;
				}
			}
		}	
	}
	return 0;
}

int debug_switch = 0;
static void auto_set_speed(long double distance,struct corner_contex corner_contex)
{
	if(debug_switch == 1)
	{
		set_boat_speed = BOAT_SPEED;
	}
	else
	{
		if(corner_contex.corner_action == 1)
		{
			#ifndef program_slow_down
			//不减速过弯，直接提前3m进行转弯
			if(distance<3.0f)
			{
				break;
			}
			#else
			if(distance<4.0f)
			{
				set_boat_speed = CURVE_SPEED;
			}
			else
			{
				set_boat_speed = BOAT_SPEED;
			}
			#endif
			

		}
		else
		{
			set_boat_speed = BOAT_SPEED;
		}
	}
}

static void auto_set_azimuth(struct track_index track_index,int azimuth,long double distance,double lastdistance,float *setvalue)
{
	int next_track_index = 0;
	int fixazimuth = 0;
	next_track_index = track_index.next_track_index;

	//小于3米时，设定方位角为指向下一个点的方向
	if(lastdistance>3.0f && distance<3.0f)
	{
		
		auto_algo_gps(currentlatitude,currentlongitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,NULL,&fixazimuth);
//		fixazimuth = azimuth;
	}
	if(distance<3.0f)
	{
		*setvalue = fixazimuth;
	}
	else
	{
		*setvalue = azimuth;
	}
	if(debug_switch == 1)
	{
		*setvalue = appvar.dst_course;
	} 
}

int debug_azimuth = 0;
static void location_thread(void *arg)
{
	long double distance =0;
	double lastdistance = 0; 
	int azimuth = 0;
	int fixazimuth = 0;
	int heading;
	long double total_distance = 0;//总长度
	double remaining_distance = 0;//剩余长度
	int azimuth_difference = 0;

	struct corner_contex corner_contex;
	struct track_index track_index;
	

	memset(&location_contex,0,sizeof(struct location_contex));
	
	float rout = 0;
	struct pid_t pid_dir;
	appvar.pid_config[0][0] = 25.00;
	appvar.pid_config[0][1] = 0.014;
	appvar.pid_config[0][2] = 90; 
	PID_struct_init(&pid_dir,Direction_pid,Vi_Position_Pid,200,200,appvar.pid_config[0][0],appvar.pid_config[0][1],appvar.pid_config[0][2]);
	pid_dir.f_pid_reset(&pid_dir,appvar.pid_config[0][0],appvar.pid_config[0][1],appvar.pid_config[0][2]);
	LOG("[AUTO:LOCATION]init\r\n");
	while(1)
	{
		//自巡航开关
		if(!location_thread_switch)
		{
			location_contex.feed_switch = 0;
			location_contex.auto_sail = 0;
			cruise_feed_control(0,0);
			curise_general_control(0,0,0);
			thread_suspend();
			auto_load_track(&track_index,&total_distance);
			PID_struct_init(&pid_dir,Direction_pid,Vi_Position_Pid,200,200,appvar.pid_config[0][0],appvar.pid_config[0][1],appvar.pid_config[0][2]);
			auto_cruise_pid_init();
			location_contex.feed_switch = 1;
			location_contex.auto_sail = 1;
		}
		
		LOG("[AUTO:LOCATION]current_track_index=%d\r\n",track_index.current_track_index);

		//第一次会将最近的点的下一个点设为目标点，后续则偏移
		auto_location_set_destination(tracks[track_index.current_track_index].latitude,tracks[track_index.current_track_index].longitude);
		/*------------------------------------------------------------------------------------------------------------------------
		转弯判断 
		------------------------------------------------------------------------------------------------------------------------*/
		if(location_contex.auto_sail != 1)
		{
			auto_calc_corner(&track_index,&corner_contex);
		}
		/*------------------------------------------------------------------------------------------------------------------------
		------------------------------------------------------------------------------------------------------------------------*/
		while(1)
		{
			if(!location_thread_switch)
			{ 
				break;
			}
			event_timed_wait(location_event, 50);//目前GPS是100ms一个数据，要注意超声波数据的有效性
			if(location_contex.feed_switch == 3)
			{
				//船停下了，这里估计是撞什么东西上了，关闭投料装置
				//！！！这里要注意，如果以后要加倒退调整船身，这个判定方式就不行，需要用两点之间距离来判断，距离在长时间不变说明船不动了
				if(currentspeed<0.2)
				{
//					cruise_feed_control(0,0);
//					location_contex.feed_switch = 1;
				}
			}
			//计算当前与目标点之间的距离和方位角
			mutex_lock(location_mutex);
			//计算出当前位置与目标点的距离和方位角 
			auto_algo_gps(currentlatitude,currentlongitude,destlatitude,destlongitude,&distance,&azimuth);
			//计算出当前目标点与下一个目标点的方位夹角

			mutex_unlock(location_mutex);
			heading = currentheading;
			static int div;
			if((div++%10)==0)
			{
				//div = 0;
				LOG_HMI("[LOCATION],L = %.2f,A = %d,\r\n",distance,azimuth);
				LOG_HMI("[LOCATION],trackindex= %d,\r\n",track_index.current_track_index);
			}
			
			
			//LOG("[LOCATION],A = %lf,\r\n",azimuth);
			if(cruise_dist_control(&currentdist) == 0)//优先判断超声波，控制船的优先级最高
			{
				//设置速度
				auto_set_speed(distance,corner_contex);
				
				
				mutex_lock(auto_boat_mutex);
				//设置航向
				auto_set_azimuth(track_index,azimuth,distance,lastdistance,&pid_dir.set);
//				pid_diff.real_out = heading;
//				rout = pid_pos(&pid_diff);
				pid_dir.f_pid_reset(&pid_dir,appvar.pid_config[0][0],appvar.pid_config[0][1],appvar.pid_config[0][2]);
				rout = pid_calc(&pid_dir,(float)heading,pid_dir.set);
				
//				if(debug_switch == 1)
//				{
//					curise_general_control(motor_speed,motor_speed,0);
//				}
//				else
				{
					//推进器控制
					cruise_gps_control_pid(distance,azimuth,heading,rout);
				}
				mutex_unlock(auto_boat_mutex);
			}


			#ifdef program1
			//过点判断
			if(auto_change_track(location_contex.auto_sail,distance,lastdistance,heading,corner_contex)==1)
			{	
				break;
			}

			lastdistance = distance;
			
			#endif

			#ifdef program0
			if(distance<1.5f)
			{
				break;
			}
			#endif
		}
		if(corner_contex.corner_action ==1)
		{
//			pid_add.Lastout = 0;
		}
		if(location_contex.feed_switch == 2)
		{
			cruise_feed_control(100,100);
			location_contex.feed_switch = 3;
		}
		//到达第一个轨迹点之后开始抛料
		if(location_contex.feed_switch == 1)
		{
			//cruise_feed_control(100,100);
			location_contex.feed_switch = 2;
		}

		if(location_contex.auto_sail == 1)
		{
			location_contex.auto_sail =2;
		}
		//开始处理下一个点
		track_index.current_track_index++;
		if(track_index.current_track_index>=track_index.tracksnum)
		{
			track_index.current_track_index = 0;
		}
		LOG("[AUTO:LOCATION]running!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
		sleep(10);
	}
}

void auto_location_resume(void)
{
	thread_resume(location_thread_t);
}
void auto_location_init(void)
{
	svc_gnss_bind(auto_get_gpslocation);
	auto_dist_bind(auto_get_dist);
	svc_compass_bind(auto_location_get_heading);
	location_event = event_create();
	location_mutex = mutex_create();
	location_thread_t = thread_create(location_thread,0,10240);
}
