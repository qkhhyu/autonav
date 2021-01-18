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
	uint8_t feed_switch;//ι��״̬ 0�رգ�1�ȴ�������2����
	uint8_t auto_sail;//Ѳ��״̬
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



//�趨Ŀ��㾭γ������
void auto_location_set_destination(long double latitude,long double longitude)
{
	mutex_lock(location_mutex);
	destlatitude = latitude;
	destlongitude = longitude;
	mutex_unlock(location_mutex);	
	LOG("[AUTO:LOCATION]set destination destlatitude=%lf destlongitude=%lf\r\n",destlatitude,destlongitude);
}

extern void auto_get_gpsspeed(struct gnss *gnss);
//��ȡ���õ�ǰλ�þ�γ������
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

//��ȡ������������Ϣ
static void auto_get_dist(struct dist *dist)
{
	memcpy(&currentdist,dist,sizeof(struct dist));
}

//������������֮��ľ���ͷ�λ��
static void auto_algo_gps(long double start_latitude,long double start_longitude,long double via_latitude,long double via_longitude,long double *distance,int *azimuth)
{	
	long double angle_A = 0;
	long double L = 0;
	long double K_ab = 0;
	L = 2*R_earth*asin(sqrt( pow(sin(angle2radian((via_latitude-start_latitude)/2)),2) + cos(angle2radian(start_latitude)) * cos(angle2radian(via_latitude)) *pow(sin(angle2radian((via_longitude-start_longitude)/2)),2) ));
	// LOG_HMI("[AUTO:LOCATION],L = %lf,\r\n",L);
	*distance = L;

	//LOG("ƽ��ֱ������ϵ��\r\n");
	angle_A = radian2angle(atan( (via_longitude-start_longitude) * cos(angle2radian(via_latitude)) / (via_latitude-start_latitude)));
		//�ж�����	
	if(via_longitude != start_longitude)
	{
		K_ab = (via_latitude - start_latitude) / (via_longitude - start_longitude);
		//LOG("K_ab = %lf\r\n", K_ab);
		if(K_ab>0)//һ������
		{
			if(via_longitude>start_longitude)//��һ����
			{
				angle_A = angle_A;
			}
			else//��������
			{
				angle_A =180 + angle_A; 
			}	
		}
		else if (K_ab<0)//��������
		{
			if(via_longitude>start_longitude)//��������
			{
				angle_A =180 + angle_A;	
			}
			else
			{
				angle_A += 360.0;
			}	
		}
		else//k=0ƽ����X��
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

//��ȡ�����������������͹켣���ܳ���
//numoftrack������켣������
//track_index������켣��
//tdistance�����ܳ�
static void auto_load_track(struct track_index *track_index,long double *tdistance)
{
	int i = 0;
	int azimuth = 0;
	long double distance =0;
	int tracksnum = 0;
	int current_track_index = 0;	
	long double distance_temp = 0; 
	double total_distance = 0;//�ܳ���

	distance_temp = 0;
	current_track_index = 0;
	tracksnum = track_read(tracks,MAX_TRACKS);//��ȡ���й켣��
	track_index->tracksnum = tracksnum;
	LOG("[AUTO:LOCATION]read tracksnum=%d\r\n",tracksnum);

	event_timed_wait(location_event, 1000);
	for(i=0;i<tracksnum;i++)//��ȡ���й켣���꣬ѡ����뵱ǰλ�������һ������
	{
		//�����뵱ǰλ�õľ��룬���������뵱ǰλ�������һ������
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
		else//���������費��Ҫ������ȵ���������յ�ǰ��λ��ȥ���������
		{
			distance_temp = distance;
			current_track_index = i;
		}
	}	
	current_track_index = (current_track_index+1)%tracksnum;//����������һ���㣬Ϊ��Ѳ���ĵ�һ���㣬���⴬������
	track_index->current_track_index = current_track_index;
	for(i=0;i<tracksnum;i++)
	{
		auto_algo_gps(tracks[i].latitude,tracks[i].longitude,tracks[(i+1)%tracksnum].latitude,tracks[(i+1)%tracksnum].longitude,&distance,&azimuth);
		//�ۼӣ�����켣���ܾ��룬Ҳ����Ϻ�����ܳ�
		total_distance = total_distance+distance;		
	}
	*tdistance = total_distance;
	track_index->last_track_index = (current_track_index+tracksnum-1)%tracksnum;
	track_index->next_track_index = (current_track_index+1)%tracksnum;
	
}

//ת���ж�
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
	int pretreatment_judgment = 0;//ת��Ƕȹ�С�жϱ��

	current_track_index = track_index->current_track_index;
	//������һ�������ǰĿ���ľ���ͷ�λ��
	last_track_index = 	track_index->last_track_index;		
	auto_algo_gps(tracks[last_track_index].latitude,tracks[last_track_index].longitude,tracks[current_track_index].latitude,tracks[current_track_index].longitude,&dist_last,&azimuth_last);
	//������һ�������ǰĿ���ľ���ͷ�λ��
	next_track_index = track_index->next_track_index;
	auto_algo_gps(tracks[current_track_index].latitude,tracks[current_track_index].longitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,&dist_next,&azimuth_next);

	//������һ����ת��ĽǶ�
	corner = ((360-azimuth_last)+azimuth_next)%360;
	if(corner<=180)
	{
		corner = corner;			
	}
	else
	{
		corner = 360-corner;
	}
	//ת��Ƕ�̫С��Ҳ����ƫת�ǲ�ֵ̫����Ҫ������ǰ���Լ����ٴ���
	if(corner>60)
	{
		pretreatment_judgment = 1;//�Ƕȹ�С����Ҫ��ǰ����ת���
	}
	else
	{
		pretreatment_judgment = 0;//��ȫ����ת��Ƕȣ�����Ҫ��ǰ����
	}
	
	if(pretreatment_judgment == 0)
	{
		//������֮�����̫����ֱ��������ѭ����һ����
		if(dist_next<4.0f)
		{
			track_index->current_track_index++;
			if(track_index->current_track_index>=track_index->tracksnum)
			{
				track_index->current_track_index = 0;
			}
			//������һ�������ǰĿ���ľ���ͷ�λ��
			//������һ���㲻��
			auto_algo_gps(tracks[last_track_index].latitude,tracks[last_track_index].longitude,tracks[current_track_index].latitude,tracks[current_track_index].longitude,&dist_last,&azimuth_last);
			//������һ�������ǰĿ���ľ���ͷ�λ��
			next_track_index = (track_index->current_track_index+1)%track_index->tracksnum;
			auto_algo_gps(tracks[current_track_index].latitude,tracks[current_track_index].longitude,tracks[next_track_index].latitude,tracks[next_track_index].longitude,&dist_next,&azimuth_next);
			//������һ����ת��ĽǶ�
			corner = ((360-azimuth_last)+azimuth_next)%360;
			if(corner<=180)
			{
				corner = corner;			
			}
			else
			{
				corner = 360-corner;
			}
			//ת��Ƕ�̫С��Ҳ����ƫת�ǲ�ֵ̫����Ҫ������ǰ���Լ����ٴ���
			if(corner>60)
			{
				pretreatment_judgment = 1;//�Ƕȹ�С����Ҫ��ǰ����ת���
			}
			else
			{
				pretreatment_judgment = 0;//��ȫ����ת��Ƕȣ�����Ҫ��ǰ����
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
	//�жϾ����Ƿ�ʼԶ��Ŀ��㣬Զ�����ʾ�Ѿ������õ�
	if(corner_contex.corner_action == 1)//Ҫת���䣬��ǰ�����õ�ѭ��
	{
		if(distance<4.0f)
		{
			if((distance-lastdistance) > 0.07f)//Զ��Ŀ�������Ϊ�Ѿ������õ�
			{
				return 1;
			}
		}
		if(distance<2.0)
		{
			return 1;
		}
	}
	else//����Ҫת���䣬����Ҫ��ǰ����
	{
		if(distance<3.0f)
		{
			//���������Ҫ�ټ�һ���жϣ��ϴ�ȥ���ţ������ٶ�̫����ʱ��һֱ�����㣬�͵����˴�ת��������
			if((distance-lastdistance) > 0.07f)//Զ��Ŀ�������Ϊ�Ѿ������õ�
			{
				return 1;
			}
		}
//		if(flag != 1)
		{
			if(lastdistance<3.0f&&distance>3.0)//��ֹ�ٶȹ�������������ת������ɴ�ת������
			{
				return 1;
			}
		}
		//����С��1��ʱҲ��Ϊ�Ѿ�����Ŀ���
		if(distance <1.0)
		{
			return 1;
		}
		if(distance<6.0f)
		{
			//Ѳ��;�У�����з��������ƣ�Ҫ��ʱ���������ⷴ����
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
				if(azimuth_difference>=90)//����������˸õ������ߣ����Ƿ��ߣ�������Ϊ�Ѿ����˸õ㣬��ֹ��ת�������
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
			//�����ٹ��䣬ֱ����ǰ3m����ת��
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

	//С��3��ʱ���趨��λ��Ϊָ����һ����ķ���
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
	long double total_distance = 0;//�ܳ���
	double remaining_distance = 0;//ʣ�೤��
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
		//��Ѳ������
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

		//��һ�λὫ����ĵ����һ������ΪĿ��㣬������ƫ��
		auto_location_set_destination(tracks[track_index.current_track_index].latitude,tracks[track_index.current_track_index].longitude);
		/*------------------------------------------------------------------------------------------------------------------------
		ת���ж� 
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
			event_timed_wait(location_event, 50);//ĿǰGPS��100msһ�����ݣ�Ҫע�ⳬ�������ݵ���Ч��
			if(location_contex.feed_switch == 3)
			{
				//��ͣ���ˣ����������ײʲô�������ˣ��ر�Ͷ��װ��
				//����������Ҫע�⣬����Ժ�Ҫ�ӵ��˵�����������ж���ʽ�Ͳ��У���Ҫ������֮��������жϣ������ڳ�ʱ�䲻��˵����������
				if(currentspeed<0.2)
				{
//					cruise_feed_control(0,0);
//					location_contex.feed_switch = 1;
				}
			}
			//���㵱ǰ��Ŀ���֮��ľ���ͷ�λ��
			mutex_lock(location_mutex);
			//�������ǰλ����Ŀ���ľ���ͷ�λ�� 
			auto_algo_gps(currentlatitude,currentlongitude,destlatitude,destlongitude,&distance,&azimuth);
			//�������ǰĿ�������һ��Ŀ���ķ�λ�н�

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
			if(cruise_dist_control(&currentdist) == 0)//�����жϳ����������ƴ������ȼ����
			{
				//�����ٶ�
				auto_set_speed(distance,corner_contex);
				
				
				mutex_lock(auto_boat_mutex);
				//���ú���
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
					//�ƽ�������
					cruise_gps_control_pid(distance,azimuth,heading,rout);
				}
				mutex_unlock(auto_boat_mutex);
			}


			#ifdef program1
			//�����ж�
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
		//�����һ���켣��֮��ʼ����
		if(location_contex.feed_switch == 1)
		{
			//cruise_feed_control(100,100);
			location_contex.feed_switch = 2;
		}

		if(location_contex.auto_sail == 1)
		{
			location_contex.auto_sail =2;
		}
		//��ʼ������һ����
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
