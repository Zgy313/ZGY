#include <stdint.h>
#include <zf_driver_pwm.h>
#include "Servo.h"



//舵机初始化
void Servo_Init(void)
{
    pwm_init(ATOM1_CH1_P33_9,50,0);

}

//舵机转向
void Set_Servo(uint8_t angle)
{
    uint16_t ServoAngle = 0;
    if((angle<=180)&&(angle>=0))
    {
        ServoAngle = (uint16_t)(250+(100*angle)/18);

    }else return ;

    pwm_set_duty(ATOM1_CH1_P33_9,ServoAngle);
}
