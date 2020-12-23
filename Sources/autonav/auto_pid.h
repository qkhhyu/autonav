#ifndef __AUTO_PID_H
#define __AUTO_PID_H


typedef struct {
    float scope;    //����޷���
    float setaim;  //Ŀ�������
    float real_out; //���������
    float Kp;   //����ϵ��
    float Ki;   //����ϵ��
    float Kd;   //΢��ϵ��
    float integral;   //������
    float err;   //��ǰ���
    float err_last;   //��һ�����
}PID_Type;

float pid_pos(PID_Type* pid);
void pid_init(PID_Type* pid);


struct pid_add{
	float scope;    //����޷���
    float SetSpeed;            //�����趨ֵ
    float ActualSpeed;        //����ʵ��ֵ
	float Lastout;
    float err;                //����ƫ��ֵ
    float err_next;            //������һ��ƫ��ֵ
    float err_last;            //��������ǰ��ƫ��ֵ
    float Kp,Ki,Kd;            //������������֡�΢��ϵ��
};

struct pid_changei{
	float scope;    //����޷���
    float SetSpeed;            //�����趨ֵ
    float ActualSpeed;        //����ʵ��ֵ
    float err;                //����ƫ��ֵ
    float err_last;            //������һ��ƫ��ֵ
    float Kp,Ki,Kd;            //������������֡�΢��ϵ��
    float voltage;             //�������ִ�����ı���
    float integral;            //�������ֵ
};

void PID_init1(struct pid_add *pid);
float PID_realize(struct pid_add *pid,int mode);

void PID_init2(struct pid_changei *pid);
float PID_realize_changei(struct pid_changei *pid);


#endif
