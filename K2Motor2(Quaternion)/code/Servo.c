#include <PID.h>
#include <stdint.h>
#include <zf_driver_pwm.h>
#include "Servo.h"

//#define BlaGain gain
//0.024
#define BlaGain 0.032
float gain=0;
uint8_t Servo=90;//�����ʼ���Ƕ�
uint8_t ServoShow1=0;
float ServoAid=0;
//�����ʼ��
void Servo_Init(void)
{
    pwm_init(ATOM1_CH1_P33_9,50,0);

}

//���ת��
void Set_Servo(float angle)
{
    uint16_t ServoAngle = 0;
    if((angle<=180.0)&&(angle>=0))
    {
        ServoAngle = 10000.0f*((0.5f+angle/90.0f)/20.0f);

    }else return ;

    pwm_set_duty(ATOM1_CH1_P33_9,ServoAngle);
}

//�������ƽ��
void Balance_Aid(void)
{
    uint8_t angle=0;

//    angle=Servo+ServoAid*BlaGain;

    angle=Servo-Roll.out*BlaGain;
//    if((angle<=105)&&(angle>=75))
//    {
//        Set_Servo(angle);
//    }
//    else if(angle<=105)
//    {
//        Set_Servo(105);
//    }else if(angle>=75)
//    {
//        Set_Servo(75);
//    }
    if(angle>0&&angle<180)
    {
        Set_Servo(angle);
    }else {
        if(angle<=0)
        {
            Set_Servo(0);
        }else
        {

            Set_Servo(180);
        }

    }

    ServoShow1=angle;

}
