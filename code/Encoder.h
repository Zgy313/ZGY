#ifndef _ENCODER_H_
#define _ENCODER_H_


#include "../libraries/zf_driver/zf_driver_exti.h"
#include "../libraries/zf_driver/zf_driver_gpio.h"
#include "Kalman.h"

//K2��ģ���� ��դ����һȦ30����  ���ٱ� 16��50����դ�����֣� ����һת����ԼZ=94����
// ����ֱ�� D=10cm T�����٣�V=(3.1415926*D)/(Z*T)   TΪ��������ļ������λΪs��,V�ĵ�λΪcm/s

#define D 10.0f    //ֱ��
#define Pi  3.1415926   //Բ����
#define Z  94    //����һת����ԼZ=94����



typedef struct
{
        int8_t DIR;             //ת������
        uint8_t Flag;           //���ٱ�־λ(1---��ʼ���٣�2---�������)
        float T;                //��������ļ��ʱ��
        float M1T;              //Ħ��1�ŵ�����������ټ�ʱ��ʼʱ��
        float M2T;              //Ħ��2�ŵ�����������ټ�ʱ��ʼʱ��
        uint8_t timeout;          //��ʱ���
        float speed;            //ת��
        float lastspeed;        //�ϴ�ת��
        KFP ENCODER;            //������ʼ���������˲�
        gpio_pin_enum PinA;     //A������
        gpio_pin_enum PinB;     //B������
        uint8_t PinAState;      //A������״̬
        uint8_t PinBState;      //B������״̬

}SPEED;


extern SPEED speed1;
extern SPEED speed2;

//���������ٳ�ʼ��
void SpeedCal_Init(SPEED *speed,gpio_pin_enum A,gpio_pin_enum B);

//�ٶȼ���
void SpeedCal(SPEED *speed);

//���ٳ�ʱ���(�ŵ�2ms��ʱ�ж���)
void Time_Out_Detect(SPEED *speed);

#endif
