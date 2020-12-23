#include <string.h>
#include <math.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "sail.h"
#include "gps.h"
#include "svc_gps.h"
#include "svc_compass.h"
#include "svc_dist.h"

#include "auto_boat.h"
#include "auto_location.h"
#include "auto_boat_cruise.h"

#define MAX_CAL_NUM	5
#define PI (3.1415926535898)
#define R_earth (6371393)

#define radian2angle(radian)	(180.0/PI*(radian))
#define angle2radian(angle)		(PI/180*(angle))

static void(*m_handler)(struct dist *dist);

thread_t auto_dist_thread_t = NULL;
event_t  auto_dist_event;

static struct dist auto_dist;


void Swap(uint16_t *a, uint16_t *b)
{
	int temp;

	temp = *a;
	*a = *b;
	*b = temp;
}

//ð������
void BubbleSort(uint16_t *inbuf, int n)
{
    int i, j;
    for(i=0; i<n; i++){
        bool flag=false;              //��ʾ����ð���Ƿ��������ı�־
        for(j=1; j<n-i; j++){         //j����ʼλ��Ϊ1����ֹλ��Ϊn-i  
            if(inbuf[j]<inbuf[j-1]){
               Swap(&inbuf[j-1], &inbuf[j]);
                flag=true;
            }
        }
        if(flag==false)             //δ������˵���Ѿ�����ֹͣ����
            return;
    }          
}

//�޷��˲�
#define MAXDIFF	200	//�������β�����ֵ���ֵ
struct dist lastdist;
void auto_diff_filt_algo(struct dist *dist)
{
	if (lastdist.right1 == 0)
	{
		lastdist.right1 = dist->right1;
	}
	if (lastdist.right2 == 0)
	{
		lastdist.right2 = dist->right2;
	}
	if (lastdist.front == 0)
	{
		lastdist.front = dist->front;
	}
	if (lastdist.back == 0)
	{
		lastdist.back = dist->back;
	}
	if((dist->right1-lastdist.right1)>0 && (dist->right1-lastdist.right1)>MAXDIFF)//����һ�����Ƚϲ�ֵ
	{
		lastdist.right1 = dist->right1;
		dist->right1 = lastdist.right1;
		
	}
	else if((lastdist.right1-dist->right1)>0 && (lastdist.right1-dist->right1)>MAXDIFF)
	{
		lastdist.right1 = dist->right1;
		dist->right1 = lastdist.right1;
		
	}
	else//���������Χ�ڣ��򱣳�ԭ���ݣ����Ҽ�¼����һ�����ݽṹ���ڣ����´μ�����
	{
		lastdist.right1 = dist->right1;
	}

	if((dist->right2-lastdist.right2)>0 && (dist->right2-lastdist.right2)>MAXDIFF)//����һ�����Ƚϲ�ֵ
	{
		lastdist.right2 = dist->right2;
		dist->right2 = lastdist.right2;
		
	}
	else if((lastdist.right2-dist->right2)>0 && (lastdist.right2-dist->right2)>MAXDIFF)
	{
		lastdist.right2 = dist->right2;
		dist->right2 = lastdist.right2;
		
	}
	else//���������Χ�ڣ��򱣳�ԭ���ݣ����Ҽ�¼����һ�����ݽṹ���ڣ����´μ�����
	{
		lastdist.right2 = dist->right2;
	}
	if((dist->front-lastdist.front)>0 && (dist->front-lastdist.front)>MAXDIFF)//����һ�����Ƚϲ�ֵ
	{
		lastdist.front = dist->front;
		dist->front = lastdist.front;
		
	}
	else if((lastdist.front-dist->front)>0 && (lastdist.front-dist->front)>MAXDIFF)
	{
		lastdist.front = dist->front;
		dist->front = lastdist.front;
		
	}
	else//���������Χ�ڣ��򱣳�ԭ���ݣ����Ҽ�¼����һ�����ݽṹ���ڣ����´μ�����
	{
		lastdist.front = dist->front;
	}
	if((dist->back-lastdist.back)>0 && (dist->back-lastdist.back)>MAXDIFF)//����һ�����Ƚϲ�ֵ
	{
		lastdist.back = dist->back;
		dist->back = lastdist.back;
		
	}
	else if((lastdist.back-dist->back)>0 && (lastdist.back-dist->back)>MAXDIFF)
	{
		lastdist.back = dist->back;
		dist->back = lastdist.back;
		
	}
	else//���������Χ�ڣ��򱣳�ԭ���ݣ����Ҽ�¼����һ�����ݽṹ���ڣ����´μ�����
	{
		lastdist.back = dist->back;
	}	
}

//����ƽ���˲��㷨������ƽ���˲��㷨��
#define MAXQUEUE 12	//	������
static uint16_t smooth_distright1[MAXQUEUE];
static uint16_t smooth_distright2[MAXQUEUE];
int smooth_index = 0;
void auto_smooth_filt_algo(struct dist *dist)
{
	int count;
	int sumright1 = 0;
	int sumright2 = 0;
	smooth_distright1[smooth_index] = dist->right1;
	smooth_distright2[smooth_index] = dist->right2;
	smooth_index++;

	if(smooth_index == MAXQUEUE)//ѭ������
	{
		smooth_index = 0;
	}
	for(count=0;count<MAXQUEUE;count++)
	{
		sumright1 += smooth_distright1[count];
		sumright2 += smooth_distright2[count];
	}
	dist->right1 = (sumright1/MAXQUEUE);
	dist->right2 = (sumright2/MAXQUEUE);

}


static uint16_t distfront[MAX_CAL_NUM];
static uint16_t distback[MAX_CAL_NUM];
static uint16_t distright1[MAX_CAL_NUM];
static uint16_t distright2[MAX_CAL_NUM];
static uint16_t dist_temp[MAX_CAL_NUM];

int filt_index = 0;

void auto_filt_algo(struct dist *dist)
{
	distfront[filt_index] = dist->front;
	distback[filt_index] = dist->back;
	distright1[filt_index] = dist->right1;
	distright2[filt_index] = dist->right2;
	
	filt_index++;
	if(filt_index == MAX_CAL_NUM)
	{
		filt_index = 0;
	}
	
	memcpy(dist_temp,distfront,sizeof(dist_temp));
	BubbleSort(dist_temp,MAX_CAL_NUM);
	dist[0].front = (dist_temp[1]+dist_temp[2]+dist_temp[3])/3;

	memcpy(dist_temp,distback,sizeof(dist_temp));
	BubbleSort(dist_temp,MAX_CAL_NUM);
	dist->back = (dist_temp[1]+dist_temp[2]+dist_temp[3])/3;
	
	memcpy(dist_temp,distright1,sizeof(dist_temp));
	BubbleSort(dist_temp,MAX_CAL_NUM);
	dist->right1 = (dist_temp[1]+dist_temp[2]+dist_temp[3])/3;
	
	memcpy(dist_temp,distright2,sizeof(dist_temp));
	BubbleSort(dist_temp,MAX_CAL_NUM);
	dist->right2 = (dist_temp[1]+dist_temp[2]+dist_temp[3])/3;
	// printf("algo throw: right1=%d right2=%d\r\n",dist.right1,dist.right2);

}

void auto_dist_bind(void(*handler)(struct dist *dist))
{
	m_handler = handler;
}

static void auto_dist_thread(void *arg)
{
	struct dist dist;
	LOG("[AUTO:DIST]init\r\n");
	while(1)
	{
		event_wait(auto_dist_event);
		memcpy(&dist,&auto_dist,sizeof(struct dist));
		auto_diff_filt_algo(&dist);
		auto_filt_algo(&dist);
		if(m_handler)
		{
			m_handler(&dist);
		}

		//LOG("[AUTO:DIST],dist3=%d, dist4=%d, dist1=%d, dist2=%d,\r\n", dist.front, dist.back, dist.right1, dist.right2);
		// mutex_lock(auto_boat_mutex);
		//
		//=cruise_dist_control(&dist);
		// mutex_unlock(auto_boat_mutex);



	}

}

//��ȡ��ǰ����������
static void auto_get_dist(struct dist *dist)
{
	memcpy(&auto_dist, dist, sizeof(struct dist));
	event_post(auto_dist_event);
}

void auto_dist_init(void)
{
	svc_dist_bind(auto_get_dist);
	auto_dist_event = event_create();
	auto_dist_thread_t = thread_create(auto_dist_thread,0,10240);
}
