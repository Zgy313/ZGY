#include <stdint.h>
#include <zf_driver_gpio.h>
#include "Motor.h"

//motor2  dir p21_5 pwm p21_4
//motor1  dir p21_3 pwm p21_2

//Ħ����ˢ�����ʼ������
void MotorInit(void)
{
    gpio_init(P21_5, GPO, 0, GPO_PUSH_PULL);//���2dir
    gpio_init(P21_3, GPO, 0, GPO_PUSH_PULL);//���1dir
    pwm_init(ATOM0_CH0_P21_2,300,0);//���1
    pwm_init(ATOM1_CH2_P21_4,300,0);//���2

}

//Ħ�е����ˢ�����������
//num:1---��ߵ����2----�ұߵ��
//dir:1----��ת��0-----��ת
//speed: �ٶȣ�Ĭ�����ȡ10000
//Ħ�����ҿ��ƣ���� 1_0 �Ұ� 0_1����һ������1�ŵ����dir,�ڶ�������2�ŵ����dir��
void MotorRun(uint8_t num ,uint8_t dir ,int16_t speed )
{
    switch(num)
    {
        case 2:
            if((dir==0)||(dir==1))
            {
                gpio_set_level(P21_5,dir);
                pwm_set_duty(ATOM1_CH2_P21_4, speed);
            }

            break;
        case 1:
            if((dir==0)||(dir==1))
            {
                gpio_set_level(P21_3,dir);
                pwm_set_duty(ATOM0_CH0_P21_2, speed);
            }
            break;
        default:
            break;


    }

}

//HIP4082����оƬ������������L298N������ʽһ��
void Motor_HIP4082Drive(uint8_t num ,uint8_t dir ,int16_t speed)
{
    if(speed>10000)
    {
        speed=10000;
    }else if(speed<=0)
    {
        speed=0;
        dir=0;
    }
    switch(num)
    {
        case 2:
            if((dir==0)||(dir==1))
            {

                gpio_set_level(P21_5,dir);
                pwm_set_duty(ATOM1_CH2_P21_4, dir>0?(10000-speed):(speed));
            }

            break;
        case 1:
            if((dir==0)||(dir==1))
            {
                gpio_set_level(P21_3,dir);
                pwm_set_duty(ATOM0_CH0_P21_2, dir>0?(10000-speed):(speed));
            }
            break;
        default:
            break;


    }
}


