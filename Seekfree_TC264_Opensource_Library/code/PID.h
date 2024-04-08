#ifndef _PID_H_
#define _PID_H_

#include "IFXPORT.h"
#include "zf_common_typedef.h"


//���ȶ���PID�ṹ�����ڴ��һ��PID������
typedef struct
{
    //����������ʽpid
    float kp,ki,kd;//����ϵ��
    float error,lastError;//���ϴ����
    float integral,maxIntegral;//���֡������޷�
    float output,maxOutput;//���������޷�
    float reference,feedback;//Ŀ��ֵ��ʵ��ֵ
    float prev_error;
    //������λ��ʽpid
    float SetValue;       //�趨ֵ
    float ActualValue;  //ʵ��ֵ
    float Kp_pos;       //����ϵ��
    float ki_pos;      //����ϵ��
    float kd_pos;      //΢��ϵ��
    float T;       //��������
    float kpout;   //�������������
    float kiout;   //�������������
    float kdout;   //΢�����������
    float ek;     //��ǰƫ��
    float ek1;     //��һ��ƫ��
    float ek2;     //���ϴ�ƫ��
    float out;    //���������
}PID;



extern PID Motor1;
extern PID Motor2;
extern PID Roll;
extern PID Yaw;
//extern u8 PIDCalFlag;
extern u8 tm;
extern u8 tr;
extern u8 tg;
extern u8 ty;
extern u16 roll_kalman;
//������PID��������ʼ��
void AllPID_Init(void);
//����ʽPID������ʼ��
void PID_Init(PID *pid,float p,float i,float d,float maxOut);
//λ��ʽpid��ʼ��
void POS_PID_Init(PID *PID,float p,float i,float d,float maxI,float maxOut);
//����ʽPID���㺯��
void PID_Calc(PID *pid,float reference,float feedback);
//λ��ʽpid
void PID_POS(PID *pid);
//PID���������
void PID_Realize(void);
//PID�������
void PID_Out(uint8_t num,int16_t speed);




#endif