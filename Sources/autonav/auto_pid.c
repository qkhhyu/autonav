#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "kernel.h"
#include "log.h"
#include "app.h"
#include "bsp.h"
#include "hmi.h"
#include "sail.h"
#include "gnss.h"

#include "auto_boat_cruise.h"
#include "auto_pid.h"

#define max(a,b)   (a>b? a:b)
#define min(a,b)   (a<b? a:b)
#define limiter(x,a,b)  (min(max(x,a),b))

//--------------------------------------------------------------------------------------------------//
//λ��ʽPID
float pid_pos(PID_Type* p)
{
    float pe, ie, de;
    float out = 0;
	float temp = 0;
	temp = ((int)((360.0-p->setaim)+p->real_out))%360;//����������λ�ǵĲ�ֵ
	if(temp>180&&temp<=360)
	{
		p->err = (360-temp);
	}
	else if(temp<=180&&temp>=0)
	{
		p->err = -temp;
	}
//    p->err = p->setaim - p->real_out;//���㵱ǰ���

    p->integral += p->err;//������

    de = p->err - p->err_last;//���΢��

    pe = p->err;
    ie = p->integral;

    p->err_last = p->err;

    out = pe * (p->Kp) + ie * (p->Ki) + de * (p->Kd);

    out = limiter(out, -p->scope, p->scope);//����޷�
    return out;
}


void pid_init(PID_Type* pid)
{
    memset(pid, 0, sizeof(PID_Type));
}
//--------------------------------------------------------------------------------------------------//
//����ʽPID
void PID_init1(struct pid_add *pid)
{
    pid->SetSpeed=0.0f;
    pid->ActualSpeed=0.0f;
	pid->Lastout = 0.0f;
    pid->err=0.0f;
    pid->err_last=0.0f;
    pid->err_next=0.0f;
    pid->Kp=0.0f;
    pid->Ki=0.0f;
    pid->Kd=0.0f;
}

float PID_realize(struct pid_add *pid,int mode)
{
	float temp = 0;
//    pid->SetSpeed=speed;
	if(mode == 0)
	{
		temp = ((int)((360.0-pid->SetSpeed)+pid->ActualSpeed))%360;//����������λ�ǵĲ�ֵ
		if(temp>180&&temp<=360)
		{
			pid->err = (360-temp);
		}
		else if(temp<=180&&temp>=0)
		{
			pid->err = -temp;
		}
	}
	else
	{
		pid->err=pid->SetSpeed-pid->ActualSpeed;
	}
	
//    pid->err=pid->SetSpeed-pid->ActualSpeed;
    float incrementSpeed=pid->Kp*(pid->err-pid->err_next)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_next+pid->err_last);
    //pid->ActualSpeed+=incrementSpeed;
    pid->err_last=pid->err_next;
    pid->err_next=pid->err;
	if(mode == 0)
	{
//		 incrementSpeed = incrementSpeed+pid->err;
//		 incrementSpeed = limiter(incrementSpeed, -pid->scope, pid->scope);//����޷�
//		 return incrementSpeed;
		pid->Lastout = pid->Lastout+incrementSpeed;
		pid->Lastout = limiter(pid->Lastout, -pid->scope, pid->scope);//����޷�
		return pid->Lastout;
	}
	else
	{
//		pid->ActualSpeed = (pid->ActualSpeed)+500;
//		pid->ActualSpeed = limiter(pid->ActualSpeed, -pid->scope, pid->scope);//����޷�
		pid->Lastout = pid->Lastout+incrementSpeed;
		pid->Lastout = limiter(pid->Lastout, -pid->scope, pid->scope);//����޷�
		return pid->Lastout;
	}
	

	
}
//----------------------------------------------------------------------------------------------------------------//
//����ֵ�PID�����㷨


void PID_init2(struct pid_changei *pid)
{
    pid->SetSpeed=0.0;
    pid->ActualSpeed=0.0;
    pid->err=0.0;
    pid->err_last=0.0;
	pid->voltage = 0;
	pid->integral = 0;
    pid->Kp=0.0;
    pid->Ki=0.0;
    pid->Kd=0.0;
}

float PID_realize_changei(struct pid_changei *pid)
{
	float temp = 0;
	float index = 0;
//	pid->SetSpeed=speed;
//	pid->err=pid->SetSpeed-pid->ActualSpeed;
	temp = ((int)((360.0-pid->SetSpeed)+pid->ActualSpeed))%360;//����������λ�ǵĲ�ֵ
	if(temp>180&&temp<=360)
	{
		pid->err = (360-temp);
	}
	else if(temp<=180&&temp>=0)
	{
		pid->err = -temp;
	}

	//����ֹ���
	if(fabs(pid->err)>120)
	{
		index=0.0;
	}
	else if(fabs(pid->err)<90)
	{
		index=1.0;
		pid->integral+=pid->err;
	}
	else
	{
		index=(120-fabs(pid->err))/15;
		pid->integral+=pid->err;
	}
	pid->voltage=pid->Kp*pid->err+index*pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);

	pid->err_last=pid->err;
	pid->ActualSpeed=pid->voltage*1.0;
	pid->ActualSpeed = limiter(pid->ActualSpeed, -pid->scope, pid->scope);//����޷�
	return pid->ActualSpeed;
}



