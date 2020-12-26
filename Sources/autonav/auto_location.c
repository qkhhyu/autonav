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
}

//获取最近的坐标点索引，和轨迹的总长度
static void auto_load_track(int *numoftrack,int *track_index,long double *tdistance)
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
	*numoftrack = tracksnum;
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
	*track_index = current_track_index;
	for(i=0;i<tracksnum;i++)
	{
		auto_algo_gps(tracks[i].latitude,tracks[i].longitude,tracks[(i+1)%tracksnum].latitude,tracks[(i+1)%tracksnum].longitude,&distance,&azimuth);
		//累加，计算轨迹的总距离，也就是虾塘的周长
		total_distance = total_distance+distance;		
	}
	*tdistance = total_distance;
	
}

int debug_azimuth = 0;
int debug_switch = 0;
static void location_thread(void *arg)
{
	long double dist_last;
	long double dist_next;
	int azimuth_last;
	int azimuth_next;
	long double distance =0;
	double lastdistance = 0; 
	int azimuth = 0;
	int fixazimuth = 0;
	int heading;
	int tracksnum = 0;
	int current_track_index = 0;
	int last_track_index = 0;
	int next_track_index = 0;
	long double total_distance = 0;//总长度
	double remaining_distance = 0;//剩余长度
	int corner = 0;
	int pretreatment_judgment = 0;//转弯角度过小判断标记
	int azimuth_difference = 0;

	
	struct pid_add pid_add;

	memset(&location_contex,0,sizeof(struct location_contex));
	
	double rout = 0;
	#ifdef PIDDCHANGEI
	struct pid_changei pid;
	PID_init2(&pid);
	appvar.pid_config[0][0] = 3.52;
	appvar.pid_config[0][1] = 0.014;
	appvar.pid_config[0][2] = 60;
	pid.Kp = appvar.pid_config[0][0];
	pid.Ki = appvar.pid_config[0][1];
	pid.Kd = appvar.pid_config[0][2];
	pid.scope = 200;
	#else

	PID_init1(&pid_add);
	appvar.pid_config[0][0] = 1.82;
	appvar.pid_config[0][1] = 0.007;
	appvar.pid_config[0][2] = 0.0;
	pid_add.Kp = appvar.pid_config[0][0];
	pid_add.Ki = appvar.pid_config[0][1];
	pid_add.Kd = appvar.pid_config[0][2];
	pid_add.scope = 200;
	#endif
	LOG("[AUTO:LOCATION]init\r\n");
	while(1)
	{
		//自巡航开关
		if(!location_thread_switch)
		{
			location_contex.feed_switch = 0;
			location_contex.auto_sail = 0;
//			PID_init1(&pid_add);
			thread_suspend();
			auto_load_track(&tracksnum,&current_track_index,&total_distance);
			auto_cruise_pid_init();
			location_contex.feed_switch = 1;
			location_contex.auto_sail = 1;
		}
		
		LOG("[AUTO:LOCATION]current_track_index=%d\r\n",current_track_index);

		//第一次会将最近的点的下一个点设为目标点，后续则偏移
		auto_location_set_destination(tracks[current_track_index].latitude,tracks[current_track_index].longitude);
		/*------------------------------------------------------------------------------------------------------------------------
		转弯判断
		------------------------------------------------------------------------------------------------------------------------*/
		//计算上一个点跟当前目标点的距离和方位角
		last_track_index = 	(current_track_index+tracksnum-1)%tracksnum;		
		auto_algo_gps(tracks[last_track_index].latitude,tracks[last_track_index].longitude,tracks[current_track_index].latitude,tracks[current_track_index].longitude,&dist_last,&azimuth_last);
		//计算下一个点跟当前目标点的距离和方位角
		next_track_index = (current_track_index+1)%tracksnum;
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
				current_track_index++;
				if(current_track_index>=tracksnum)
				{
					current_track_index = 0;
				}
				//计算上一个点跟当前目标点的距离和方位角
				//这里上一个点不变
				auto_algo_gps(tracks[last_track_index].latitude,tracks[last_track_index].longitude,tracks[current_track_index].latitude,tracks[current_track_index].longitude,&dist_last,&azimuth_last);
				//计算下一个点跟当前目标点的距离和方位角
				next_track_index = (current_track_index+1)%tracksnum;
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
//					pid_add.Lastout = 0.0;
				}
				else
				{
					pretreatment_judgment = 0;//安全运行转弯角度，不需要提前处理
				}
			}
		}
		
		/*------------------------------------------------------------------------------------------------------------------------
		------------------------------------------------------------------------------------------------------------------------*/
		#ifdef program_pid
		
		//auto_cruise_pid_init();
		#endif
		//event_timed_wait(location_event, 1000);
		//cruise_gps_control_pid(0,0,0,0);
		while(1)
		{
			if(!location_thread_switch)
			{ 
				break;
			}
			event_timed_wait(location_event, 50);//目前GPS是100ms一个数据，要注意超声波数据的有效性
			if(location_contex.feed_switch == 2)
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
				LOG_HMI("[LOCATION],trackindex= %d,\r\n",current_track_index);
			}
			
			
			//LOG("[LOCATION],A = %lf,\r\n",azimuth);
//			if(cruise_dist_control(&currentdist) == 0)//优先判断超声波，控制船的优先级最高
//			{
				if(debug_switch == 1)
				{
					set_boat_speed = BOAT_SPEED;
				}
				else
				{
					if(pretreatment_judgment == 1)
					{
						#ifndef program_slow_down
						//不减速过弯，直接提前3m进行转弯
						if(distance<3.0f)
						{
							break;
						}
						#else
						if(distance<3.0f)
						{
							break;
						}
						else if(distance<4.5f)
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
				
				
				
				mutex_lock(auto_boat_mutex);
				#ifdef program_pid
				//小于3米时，设定方位角为指向下一个点的方向
				if(lastdistance>3.0f && distance<3.0f)
				{
					
					auto_algo_gps(currentlatitude,currentlongitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,NULL,&fixazimuth);
//					fixazimuth = azimuth;
				}
				if(distance<3.0f)
				{
//					pid_diff.setaim = fixazimuth;
					#ifdef PIDDCHANGEI
					pid.SetSpeed = fixazimuth;
					#else
					pid_add.SetSpeed = fixazimuth;
					#endif
				}
				else
				{
//					pid_diff.setaim = azimuth;
					#ifdef PIDDCHANGEI
					pid.SetSpeed = azimuth;
					#else
					pid_add.SetSpeed = azimuth;
					#endif
				}
				if(debug_switch == 1)
				{
//					pid_diff.setaim = appvar.dst_course;
					#ifdef PIDDCHANGEI
					pid.SetSpeed = appvar.dst_course;
					#else
					// pid_add.SetSpeed = azimuth;
					pid_add.SetSpeed = appvar.dst_course;
					#endif
				} 
//				pid_diff.real_out = heading;
//				rout = pid_pos(&pid_diff);
				#ifdef PIDDCHANGEI
				pid.Kp = appvar.pid_config[0][0];
				pid.Ki = appvar.pid_config[0][1];
				pid.Kd = appvar.pid_config[0][2];
				pid.ActualSpeed = heading;
				rout = PID_realize_changei(&pid);
				#else
				pid_add.Kp = appvar.pid_config[0][0];
				pid_add.Ki = appvar.pid_config[0][1];
				pid_add.Kd = appvar.pid_config[0][2];
				pid_add.ActualSpeed = heading;
				rout = PID_realize(&pid_add,0);
				#endif
				
				
				cruise_gps_control_pid(distance,azimuth,heading,rout);
				#elif program_pid_1
				cruise_gps_control_pid_1(distance,azimuth,heading,rout);
				#else
				cruise_gps_control(distance,azimuth,heading);
				#endif
				mutex_unlock(auto_boat_mutex);
//			}


			#ifdef program1
			//判断距离是否开始远离目标点，远离则表示已经经过该点
			if(pretreatment_judgment == 1)//要转大弯，提前结束该点循迹
			{
				if(distance<4.0f)
				{
					if((distance-lastdistance) > 0.07f)//远离目标点则认为已经经过该点
					{
						break;
					}
				}
				if(distance<3.0)
				{
					break;
				}
			}
			else//不需要转大弯，则不需要提前处理
			{
				if(distance<3.0f)
				{
					//这里可能需要再加一层判断，上次去厦门，这里速度太慢的时候一直不满足，就导致了打转的现象发生
					if((distance-lastdistance) > 0.07f)//远离目标点则认为已经经过该点
					{
						break;
					}
				}
				if(lastdistance<3.0f&&distance>3.0)//防止速度过慢零界点来回跳转，会造成打转的现象！
				{
					break;
				}
				//距离小于1米时也认为已经到达目标点
				if(distance <1.0)
				{
					break;
				}
				if(distance<6.0f)
				{
					//巡航途中，如果有反向航行趋势，要即时调整，避免反向航线
					if(location_contex.auto_sail ==2)
					{
						azimuth_difference = ((360-heading)+azimuth_last)%360;
						if(azimuth_difference<=180)
						{
							azimuth_difference = azimuth_difference;
						}
						else
						{
							azimuth_difference = 360-azimuth_difference;
						}
						if(azimuth_difference>=90)
						{
							break;
						}
					}
				}	
			}

			lastdistance = distance;
			
			#endif

			#ifdef program0
			if(distance<1.5f)
			{
				break;
			}
			#endif
			// if((((360-heading)+azimuth_last)%360) >135)//夹角反向过大，判断已经路过了
			// {
			// 	break;
			// }
		}
		if(pretreatment_judgment ==1)
		{
//			pid_add.Lastout = 0;
		}
		//到达第一个轨迹点之后开始抛料
		if(location_contex.feed_switch == 1)
		{
			cruise_feed_control(20,20);
			location_contex.feed_switch = 2;
		}
		if(location_contex.auto_sail == 1)
		{
			location_contex.auto_sail =2;
		}
		//开始处理下一个点
		current_track_index++;
		if(current_track_index>=tracksnum)
		{
			current_track_index = 0;
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
