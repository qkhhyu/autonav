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

//余弦定理求角度
double law_of_cosines(double side_b,double side_c,double subtense_a)
{
	long double angle_A;
	long double cosA;
	double a,b,c;
	a = subtense_a;
	b = side_b;
	c = side_c;

	cosA = (pow(b,2)+pow(c,2)-pow(a,2)) /2*b*c;
	angle_A = radian2angle(acosf(cosA));
	return (double)angle_A;
}

//勾股定理
double pythagorean_theorem(double side_a,double side_b)
{
	double a,b,c;
	a = side_a;
	b = side_b;

	c = sqrt(a*a+b*b);

	return c;
}

//海伦公式计算三角形面积
static double auto_cal_area(double side_a,double side_b,double side_c)
{
	double a,b,c,p,area;
	a = side_a;
	b = side_b;
	c = side_c;
	p = 0;
	area = 0;

	if(a+b>c && a+c>b && b+c>a)	//构成三角形的必要条件
	{
		p = (a+b+c)/2.0;
		area = sqrt(p*(p-a)*(p-b)*(p-c));//海伦公式
	}
	else
	{

	}
	return area;

}

static double auto_cal_distance(long double start_latitude,long double start_longitude,long double via_latitude,long double via_longitude)
{

	long double L = 0;

	L = 2*R_earth*asin(sqrt( pow(sin(angle2radian((via_latitude-start_latitude)/2)),2) + cos(angle2radian(start_latitude)) * cos(angle2radian(via_latitude)) *pow(sin(angle2radian((via_longitude-start_longitude)/2)),2) ));
	// LOG_HMI("[AUTO:LOCATION],L = %lf,\r\n",L);
	return (double)L;
}

static int auto_cal_azimuth(long double start_latitude,long double start_longitude,long double via_latitude,long double via_longitude)
{
	long double angle_A = 0;
	long double K_ab = 0;
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
	return (int)angle_A;
}

//计算两个坐标之间的距离和方位角
static void auto_algo_gps(long double start_latitude,long double start_longitude,long double via_latitude,long double via_longitude,double *distance,int *azimuth)
{	
	*distance = auto_cal_distance(start_latitude,start_longitude,via_latitude,via_longitude);
	*azimuth = auto_cal_azimuth(start_latitude,start_longitude,via_latitude,via_longitude);	
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
	double dist_last;	//上一个轨迹点与当前点的距离
	double dist_next;	//当前轨迹点与下一个点的距离
	uint8_t corner_action;
};

struct track_index
{
	int tracksnum;
	int current_track_index;
	int last_track_index;
	int next_track_index;
};

struct calc_contex
{
	double distance;		//当前位置与目标点的距离
	double lastdistance;	//上一次位置与目标点的距离

	double distance_away;	//远离上一个点的距离，即当前位置与上一个点的距离
};

//获取最近的坐标点索引，和轨迹的总长度
//numoftrack：输出轨迹点总数
//track_index：最近轨迹点
//tdistance：总周长
static void auto_load_track(struct track_index *track_index,long double *tdistance)
{
	int i = 0;
	int azimuth = 0;
	double distance =0;
	int tracksnum = 0;
	int current_track_index = 0;	
	long double distance_temp = 0;
	double total_distance = 0;//总长度

	double side_a;
	double side_b;
	double side_c;
	double angleA;

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

	//计算当前位置与最近点下一个点的距离
	side_a = auto_cal_distance(currentlatitude,currentlongitude,tracks[(current_track_index+1)%tracksnum].latitude,tracks[(current_track_index+1)%tracksnum].longitude);
	//计算最近点和下一个点的距离
	side_b = auto_cal_distance(tracks[current_track_index].latitude,tracks[current_track_index].longitude,tracks[(current_track_index+1)%tracksnum].latitude,tracks[(current_track_index+1)%tracksnum].longitude);
	//当前位置与最近点的距离
	side_c = distance_temp;
	//计算当前位置与最近点的夹角
	angleA = law_of_cosines(side_b,side_c,side_a);

	//判断当前点与最近点位置关系，防止船第一个点逆向航行
	if(angleA>=90.0)
	{
		current_track_index = current_track_index;
	}
	else
	{
		current_track_index = (current_track_index+1)%tracksnum;//最近距离的下一个点，为自巡航的第一个点，避免船反向航行
	}

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
	double dist_last;
	double dist_next;
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
	corner_contex->dist_last = dist_last;
	corner_contex->dist_next = dist_next;
}

static int auto_change_track(uint8_t flag,struct calc_contex calc_contex,int heading,struct corner_contex corner_contex)
{
	//判断距离是否开始远离目标点，远离则表示已经经过该点
	if(corner_contex.corner_action == 1)//要转大弯，提前结束该点循迹
	{
		if(calc_contex.distance<4.0f)
		{
			if((calc_contex.distance-calc_contex.lastdistance) > 0.07f)//远离目标点则认为已经经过该点
			{
				return 1;
			}
		}
		if(calc_contex.distance<2.0)
		{
			return 1;
		}
	}
	else//不需要转大弯，则不需要提前处理
	{
		if(calc_contex.distance<3.0f)
		{
			//这里可能需要再加一层判断，上次去厦门，这里速度太慢的时候一直不满足，就导致了打转的现象发生
			if((calc_contex.distance-calc_contex.lastdistance) > 0.07f)//远离目标点则认为已经经过该点
			{
				return 1;
			}
		}
//		if(flag != 1)
		{
			if(calc_contex.lastdistance<3.0f&&calc_contex.distance>3.0)//防止速度过慢零界点来回跳转，会造成打转的现象！
			{
				return 1;
			}
		}
		//距离小于1米时也认为已经到达目标点
		if(calc_contex.distance <1.0)
		{
			return 1;
		}
		if(calc_contex.distance<6.0f)
		{
			if(calc_contex.distance_away>corner_contex.dist_last)
			{
				return 1;
			}
		}	
	}
	return 0;
}

int debug_switch = 0;

static void auto_set_speed(double speed)
{
	set_boat_speed = speed;
}

static void auto_calc_speed(int state_machine,long double distance,struct corner_contex corner_contex)
{

	if(state_machine == STATE_AUTO_SAILING)
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
				auto_set_speed(CURVE_SPEED);
			}
			else
			{
				auto_set_speed(BOAT_SPEED);
			}
			#endif
			

		}
		else
		{
			auto_set_speed(BOAT_SPEED);
		}
	}
	else if(state_machine == STATE_AUTO_PARKED)
	{
		if(distance<=3.0f)
		{
			auto_set_speed(0);
		}
	}
}

static void auto_update_azimuth(struct track_index track_index,int azimuth,long double distance,double lastdistance,float *setvalue)
{
	int next_track_index = 0;
	int fixazimuth = 0;
	next_track_index = track_index.next_track_index;
	
	if(distance<2.5f)
	{
		auto_algo_gps(currentlatitude,currentlongitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,NULL,&fixazimuth);
		*setvalue = fixazimuth;
	}
	//小于3米时，设定方位角为指向下一个点的方向	
	else if(lastdistance<3.0f && distance<3.0f)
	{
		auto_algo_gps(currentlatitude,currentlongitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,NULL,&fixazimuth);
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

//设置电机/推进器的转速
static void auto_set_sailspeed(int16_t speed)
{
	motor_speed = speed;
}

//原地转圈，校准电子罗盘可调用
static void auto_circle()
{
	cruise_gps_control_pid(1000,1000,200);
}

//调整姿态
//当前目的是为了原地调整船身方向，方便停泊或者特殊航行方式调用
static void auto_adjust_attitude(struct pid_t *pid,int Azimuth)
{
	float rout = 0;
	// auto_set_sailspeed(0);

	rout = pid_calc(pid,(float)currentheading,(float)Azimuth);
	cruise_gps_control_pid(0,0,rout);
}

//轨迹点导航
static void auto_navi_dest(int state_machine,struct corner_contex corner_contex,struct calc_contex *calc_contex,long double delatitude,long double delongitude,struct pid_t *pid,float *rout)
{
	int azimuth = 0;
	int heading = 0; 
	float calcout = 0.0;
	//计算出当前位置与目标点的距离和方位角 
	mutex_lock(location_mutex);
	auto_algo_gps(currentlatitude,currentlongitude,delatitude,delongitude,&calc_contex->distance,&azimuth);
	mutex_unlock(location_mutex);
	heading = currentheading;
	pid->set = azimuth;
	static int div;
	if((div++%10)==0)
	{
		//div = 0;
		LOG_HMI("[LOCATION],L = %.2f,A = %d,\r\n",calc_contex->distance,azimuth);
	}
	//根据距离设置速度
	auto_calc_speed(state_machine,calc_contex->distance,corner_contex);

	mutex_lock(auto_boat_mutex);

	pid->f_pid_reset(pid,appvar.pid_config[0][0],appvar.pid_config[0][1],appvar.pid_config[0][2]);
	calcout = pid_calc(pid,(float)heading,pid->set);
	*rout = calcout;
	//推进器控制
	// cruise_gps_control_pid(motor_speed,motor_speed,rout);
	mutex_unlock(auto_boat_mutex);
}

//自动停泊
static void auto_parked(long double latitude,long double longitude,struct calc_contex *calc_contex,struct pid_t *pid,float *rout)
{
	struct corner_contex corner_contex;
	//设置停泊点，船将朝着这个点方向停泊
	auto_location_set_destination(latitude,longitude);
	
	while(1)
	{
		auto_navi_dest(STATE_AUTO_PARKED,corner_contex,calc_contex,destlatitude,destlongitude,pid,rout);
		if(calc_contex->distance<=3)
		{
			break;
		}
		cruise_gps_control_pid(motor_speed,motor_speed,*rout);

	}
	
}

//自动循迹航行
static void auto_track_sailing(struct calc_contex *calc_contex,struct track_index *track_index,struct corner_contex *corner_contex,struct pid_t *pid)
{
	int azimuth = 0;
	float rout = 0;
	int heading = 0;
	LOG("[AUTO:LOCATION]current_track_index=%d\r\n",track_index->current_track_index);

	//第一次会将最近的点的下一个点设为目标点，后续则偏移
	auto_location_set_destination(tracks[track_index->current_track_index].latitude,tracks[track_index->current_track_index].longitude);
	/*------------------------------------------------------------------------------------------------------------------------
	转弯判断 
	------------------------------------------------------------------------------------------------------------------------*/
	if(location_contex.auto_sail != 1)
	{
		auto_calc_corner(track_index,corner_contex);
	}
	/*------------------------------------------------------------------------------------------------------------------------
	------------------------------------------------------------------------------------------------------------------------*/
	while(1)
	{
		if(!location_thread_switch)
		{ 
			break;
		}
		event_timed_wait(location_event, 100);//目前GPS是100ms一个数据，要注意超声波数据的有效性
		if(location_contex.feed_switch == 3)
		{
			//船停下了，这里估计是撞什么东西上了，关闭投料装置
			//！！！这里要注意，如果以后要加倒退调整船身，这个判定方式就不行，需要用两点之间距离来判断，距离在长时间不变说明船不动了
			if(currentspeed<0.2f)
			{
//					cruise_feed_control(0,0);
//					location_contex.feed_switch = 1;
			}
		}
		//计算当前与目标点之间的距离和方位角
		mutex_lock(location_mutex);
		//计算出当前位置与目标点的距离和方位角 
		auto_algo_gps(currentlatitude,currentlongitude,destlatitude,destlongitude,&calc_contex->distance,&azimuth);
		//计算出当前目标点与下一个目标点的方位夹角

		mutex_unlock(location_mutex);
		heading = currentheading;
		static int div;
		if((div++%10)==0)
		{
			//div = 0;
			LOG_HMI("[LOCATION],L = %.2f,A = %d,\r\n",calc_contex->distance,azimuth);
			LOG_HMI("[LOCATION],trackindex= %d,\r\n",track_index->current_track_index);
		}
		
		
		//LOG("[LOCATION],A = %lf,\r\n",azimuth);
		if(cruise_dist_control(&currentdist) == 0)//优先判断超声波，控制船的优先级最高
		{
			//根据距离设置速度
			auto_calc_speed(STATE_AUTO_SAILING,calc_contex->distance,*corner_contex);
			
			
			mutex_lock(auto_boat_mutex);
			//设置航向
			auto_update_azimuth(*track_index,azimuth,calc_contex->distance,calc_contex->lastdistance,&pid->set);
//				pid_diff.real_out = heading;
//				rout = pid_pos(&pid_diff);
			pid->f_pid_reset(pid,appvar.pid_config[0][0],appvar.pid_config[0][1],appvar.pid_config[0][2]);
			rout = pid_calc(pid,(float)heading,pid->set);
			
//				if(debug_switch == 1)
//				{
//					curise_general_control(motor_speed,motor_speed,0);
//				}
//				else
			{
				//推进器控制
				cruise_gps_control_pid(motor_speed,motor_speed,rout);
			}
			mutex_unlock(auto_boat_mutex);
		}

		calc_contex->distance_away = auto_cal_distance(currentlatitude,currentlongitude,tracks[track_index->last_track_index].latitude,tracks[track_index->last_track_index].longitude);
		#ifdef program1
		//过点判断
		if(auto_change_track(location_contex.auto_sail,*calc_contex,heading,*corner_contex)==1)
		{	
			break;
		}

		calc_contex->lastdistance = calc_contex->distance;
		
		#endif

		#ifdef program0
		if(distance<1.5f)
		{
			break;
		}
		#endif
	}
	if(corner_contex->corner_action ==1)
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
	track_index->current_track_index++;
	if(track_index->current_track_index>=track_index->tracksnum)
	{
		track_index->current_track_index = 0;
	}
			
}

int debug_azimuth = 0;
static void location_thread(void *arg)
{
	long double total_distance = 0;//总长度
//	double remaining_distance = 0;//剩余长度

	struct corner_contex corner_contex;
	struct track_index track_index;
	struct calc_contex calc_contex;
	

	memset(&location_contex,0,sizeof(struct location_contex));

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
			//投料关闭
			cruise_feed_control(0,0);
			//动力归零
			curise_general_control(0,0,0);
			thread_suspend();
			//读取轨迹点
			auto_load_track(&track_index,&total_distance);
			//初始化PID参数
			PID_struct_init(&pid_dir,Direction_pid,Vi_Position_Pid,200,200,appvar.pid_config[0][0],appvar.pid_config[0][1],appvar.pid_config[0][2]);
			auto_cruise_pid_init();
			location_contex.feed_switch = 1;
			location_contex.auto_sail = 1;
		}
		//自巡航
		auto_track_sailing(&calc_contex,&track_index,&corner_contex,&pid_dir);
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
