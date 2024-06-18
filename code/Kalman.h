#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "IFXPORT.h"
#include "zf_common_typedef.h"

//��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
//��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��


typedef struct
{
    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;//��ǰ����Э���� ��ʼ��ֵΪ0
    float out;//�������˲������ ��ʼ��ֵΪ0
    float Kg;//���������� ��ʼ��ֵΪ0
    float Q;//��������Э���� ��ʼ��ֵΪ0.001
    float R;//�۲�����Э���� ��ʼ��ֵΪ0.543
}KFP;//Kalman Filter parameter

extern KFP gyro_z_str,Motor1_Str,Motor2_Str;
extern KFP Motor;
//�������˲���ʼ������
void Kalman_Init(void);
//��ȡ�������ںϺ������
float Kalman_getAngle(float dt,float Gyro_rate,float Acc_Angle);
//һά�������˲�
float kalmanFilter(KFP *kfp,float input);

uint8_t Zero_DevCal(float *Dev,uint16_t num);
#endif
