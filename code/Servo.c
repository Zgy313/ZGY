#include <stdint.h>
#include <zf_driver_pwm.h>
#include "Cpu1Init.h"
#include "Servo.h"


//-0.12
#define BlaGain 0.8
float gain=0;
//#define BlaGain gain

uint8_t ServoShow1=0;
float ServoAid=0;
//舵机初始化
void Servo_Init(void)
{
    pwm_init(ATOM1_CH1_P33_9,50,0);

}

//舵机转向
void Set_Servo(float angle)
{
    uint16_t ServoAngle = 0;
    if((angle<=180.0)&&(angle>=0))
    {
        ServoAngle = 10000.0f*((0.5f+angle/90.0f)/20.0f);

    }else return ;

    pwm_set_duty(ATOM1_CH1_P33_9,ServoAngle);
}

//舵机辅助平衡
void Balance_Aid(void)
{
    uint8_t angle=0;


//    angle=Servo+ServoAid*BlaGain;

    angle=Servo-(Roll.ActualValue-Zero)*BlaGain;
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
            Set_Servo(1);
        }else
        {

            Set_Servo(179);
        }

    }

    ServoShow1=angle;


}
