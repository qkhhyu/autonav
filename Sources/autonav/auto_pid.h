#ifndef __AUTO_PID_H__
#define __AUTO_PID_H__



/* Includes ------------------------------------------------------------------*/
#include <math.h>



/*������ʼ��--------------------------------------------------------------*/

enum
{
	LLAST	= 0,//���ϴ�
	LAST 	= 1,//�ϴ�
	NOW 	= 2,//����

	Position_Pid,//λ��ʽ
	Delta_Pid,//����ʽ
	Vi_Position_Pid,//�����λ����

	Direction_pid,
	Speed_pid,
};



struct pid_t
{
  float Kp;
  float Ki;
  float Kd;

  float set;				//Ŀ��ֵ
  float get;				//����ֵ
  float err[3];			//���ֵ,����NOW�� LAST�� LLAST���ϴ�

  float integra_index;  //�����ϵ��
    
  float pout;				//p���
  float iout;				//i���
  float dout;				//d���    
  float out;			  //pid���


  float pos_out;				//����λ��ʽ���
  float last_pos_out;			//�ϴ����
  float delta_out;				//��������ֵ
  float last_delta_out;			//��������ʽ��� = last_delta_out + delta_u

  float max_err;

  uint32_t pid_object;    //���ƶ���
  uint32_t pid_mode;      //���㹫ʽ
  uint32_t MaxOutput;				//����޷�
  uint32_t IntegralLimit;		//�����޷�
       
       
  void (*f_param_init)(struct pid_t *pid,  //PID������ʼ��
					uint32_t pid_object,
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);		//pid���������޸�

};

//������������
void PID_struct_init(struct pid_t* pid,uint32_t object,uint32_t mode,uint32_t maxout,uint32_t intergral_limit,float kp, float 	ki, float kd);
float pid_calc(struct pid_t* pid,float get, float set);
#endif

