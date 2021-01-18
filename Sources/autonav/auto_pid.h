#ifndef __AUTO_PID_H__
#define __AUTO_PID_H__



/* Includes ------------------------------------------------------------------*/
#include <math.h>



/*参数初始化--------------------------------------------------------------*/

enum
{
	LLAST	= 0,//上上次
	LAST 	= 1,//上次
	NOW 	= 2,//本次

	Position_Pid,//位置式
	Delta_Pid,//增量式
	Vi_Position_Pid,//变积分位置试

	Direction_pid,
	Speed_pid,
};



struct pid_t
{
  float Kp;
  float Ki;
  float Kd;

  float set;				//目标值
  float get;				//测量值
  float err[3];			//误差值,包含NOW， LAST， LLAST上上次

  float integra_index;  //变积分系数
    
  float pout;				//p输出
  float iout;				//i输出
  float dout;				//d输出    
  float out;			  //pid输出


  float pos_out;				//本次位置式输出
  float last_pos_out;			//上次输出
  float delta_out;				//本次增量值
  float last_delta_out;			//本次增量式输出 = last_delta_out + delta_u

  float max_err;

  uint32_t pid_object;    //控制对象
  uint32_t pid_mode;      //计算公式
  uint32_t MaxOutput;				//输出限幅
  uint32_t IntegralLimit;		//积分限幅
       
       
  void (*f_param_init)(struct pid_t *pid,  //PID参数初始化
					uint32_t pid_object,
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);		//pid三个参数修改

};

//函数声明部分
void PID_struct_init(struct pid_t* pid,uint32_t object,uint32_t mode,uint32_t maxout,uint32_t intergral_limit,float kp, float 	ki, float kd);
float pid_calc(struct pid_t* pid,float get, float set);
#endif

