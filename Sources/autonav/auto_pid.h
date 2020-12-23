#ifndef __AUTO_PID_H
#define __AUTO_PID_H


typedef struct {
    float scope;    //输出限幅量
    float setaim;  //目标输出量
    float real_out; //反馈输出量
    float Kp;   //比例系数
    float Ki;   //积分系数
    float Kd;   //微分系数
    float integral;   //误差积分
    float err;   //当前误差
    float err_last;   //上一次误差
}PID_Type;

float pid_pos(PID_Type* pid);
void pid_init(PID_Type* pid);


struct pid_add{
	float scope;    //输出限幅量
    float SetSpeed;            //定义设定值
    float ActualSpeed;        //定义实际值
	float Lastout;
    float err;                //定义偏差值
    float err_next;            //定义上一个偏差值
    float err_last;            //定义最上前的偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
};

struct pid_changei{
	float scope;    //输出限幅量
    float SetSpeed;            //定义设定值
    float ActualSpeed;        //定义实际值
    float err;                //定义偏差值
    float err_last;            //定义上一个偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float voltage;             //定义控制执行器的变量
    float integral;            //定义积分值
};

void PID_init1(struct pid_add *pid);
float PID_realize(struct pid_add *pid,int mode);

void PID_init2(struct pid_changei *pid);
float PID_realize_changei(struct pid_changei *pid);


#endif
