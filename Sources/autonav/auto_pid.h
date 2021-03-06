#ifndef __AUTO_PID_H__
#define __AUTO_PID_H__


#include <math.h>

enum
{
	LLAST	= 0,        //上上次
	LAST 	= 1,        //上次
	NOW 	= 2,        //本次

	Position_Pid,     	//位置式
	Delta_Pid,        	//增量式
	Vi_Position_Pid,  	//变积分位置试

	Direction_pid,    	//方向PID
	Speed_pid,        	//速度PID
};

struct pid_t
{
  float Kp;
  float Ki;
  float Kd;

  float set;						//目标值
  float get;						//测量值
  float err[3];						//误差值,包含NOW， LAST， LLAST上上次

  float integra_index;    			//变积分系数

  float pout;				      	//p输出
  float iout;             			//i输出
  float dout;				      	//d输出    
  float out;			        	//pid输出


  float pos_out;			 	  	//本次位置式输出
  float last_pos_out;		  		//上次输出
  float delta_out;			  		//本次增量值
  float last_delta_out;   			//本次增量式输出 = last_delta_out + delta_u

  float max_err;

  uint32_t pid_object;    			//控制对象
  uint32_t pid_mode;      			//计算公式
  float MaxOutput;				//输出限幅
  float IntegralLimit;			//积分限幅
      
      
  void (*f_param_init)(struct pid_t *pid,                               	//PID参数初始化
        uint32_t pid_object,
                  uint32_t pid_mode,
                  float maxOutput,
                  float integralLimit,
                  float p,
                  float i,
                  float d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);		//pid三个参数修改

};

void PID_struct_init(struct pid_t* pid,uint32_t object,uint32_t mode,float maxout,float intergral_limit,float kp, float 	ki, float kd);
float pid_calc(struct pid_t* pid,float get, float set);
#endif

