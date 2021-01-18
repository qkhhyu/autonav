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

/*!
 *  @brief      pid参数初始化
 *  @param      mode 模式  maxput 最大输出 Kp Ki Kd  pid参数
 *  @since      v1.0
 *  @note       
 *  Sample usage:      pid_init(pid_t *pid, Delta_Pid, 100,70,2,0.1,1);    //初始化 pid参数 为增量式输出  最大输出值为100 积分限幅为70  P为2 I为0.1 D为1
 */
static void pid_init(struct pid_t *pid,uint32_t object,uint32_t mode,uint32_t maxout,uint32_t intergral_limit,float kp,float ki,float kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_object = object;
    pid->pid_mode = mode;
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd; 
}
/*!
 *  @brief      pid中途调参
 *  @param      
 *  @since      v1.0
 *  @note       
 *  Sample usage:    pid_reset(&pidsd[1],1,2,3);    //pid调参，电机2更新p为1 i为2  d为3
 */

/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(struct pid_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/*!
 *  @brief      pid计算
 *  @param    get 实际速度   set预期速度
 *  @since      v1.0
 *  @note       
 */

float pid_calc(struct pid_t* pid,float get, float set)
{
    pid->get = get;
    pid->set = set;
    if(pid->pid_object == Direction_pid)
    {
        pid->err[NOW] = ((int)((360.0 - pid->set) + pid->get)) % 360;//计算两个方位角的差值
        if(pid->err[NOW]>180.0 && pid->err[NOW]<=360.0)
        {
           pid->err[NOW] = 360.0 - pid->err[NOW]; 
        }
        else
        {
            pid->err[NOW] = -pid->err[NOW];
        }  
    }
    if(pid->pid_object == Speed_pid)
    {
        pid->err[NOW] = pid->set - pid->get;	//set - get
    }
    

    if(pid->pid_mode == Position_Pid) //位置式pid
    {
        pid->pout = pid->Kp * pid->err[NOW];
        pid->iout += pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - pid->err[LAST] );
        pid->iout = limiter(pid->iout,-pid->IntegralLimit,pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;

        pid->pos_out = limiter(pid->pos_out,-pid->MaxOutput,pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == Delta_Pid)//增量式P
    {
        pid->pout = pid->Kp * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        pid->iout = limiter(pid->iout,-pid->IntegralLimit,pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->out;

        pid->delta_out = limiter(pid->delta_out,-pid->MaxOutput,pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    else if(pid->pid_mode == Vi_Position_Pid)
    {
        if(pid->pid_object == Direction_pid)
        {
            if(fabs(pid->err[NOW]) > 120)
            {
                pid->integra_index = 0.0;
            }
            else if(fabs(pid->err[NOW])<90)
            {
                pid->integra_index = 1.0;
            }
            else
            {
                pid->integra_index = (120-fabs(pid->err[NOW])) /15;
            }
        }

        pid->pout = pid->Kp * pid->err[NOW];
        pid->iout += pid->integra_index * pid->Ki *pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - pid->err[LAST]);
        pid->pos_out =  pid->pout + pid->iout + pid->dout;

        pid->pos_out = limiter(pid->pos_out,-pid->MaxOutput,pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];

    return ((pid->pid_mode==Delta_Pid) ? pid->delta_out : pid->pos_out);
}

/*!
 *  @brief      pid参数初始化
 *  @param      mode 模式  maxput 最大输出 Kp Ki Kd  pid参数
 *  @since      v1.0
 *  @note       
 *  Sample usage:      pid_init(pid_t *pid, Delta_Pid, 100,70,2,0.1,1);    //初始化 pid参数 为增量式输出  最大输出值为100 积分限幅为70  P为2 I为0.1  D为1
 */
void PID_struct_init(struct pid_t* pid,uint32_t object,uint32_t mode,uint32_t maxout,uint32_t intergral_limit,float kp, float 	ki, float kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_init;
    pid->f_pid_reset = pid_reset;
		
    /*init pid param */
    pid->f_param_init(pid,object,mode,maxout,intergral_limit,kp,ki,kd);
}

