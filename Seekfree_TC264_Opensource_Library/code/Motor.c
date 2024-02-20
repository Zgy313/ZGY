#include <stdint.h>
#include <zf_driver_gpio.h>
#include "Motor.h"

//motor2  dir p21_5 pwm p21_4
//motor1  dir p21_3 pwm p21_2

//摩托有刷电机初始化函数
void MotorInit(void)
{
    gpio_init(P21_5, GPO, 1, GPO_PUSH_PULL);//电机2dir
    gpio_init(P21_3, GPO, 1, GPO_PUSH_PULL);//电机1dir
    pwm_init(ATOM0_CH0_P21_2,17000,0);//电机1
    pwm_init(ATOM1_CH2_P21_4,17000,0);//电机2

}

//摩托电机有刷电机驱动函数
//num:1---左边电机，2----右边电机
//dir:1----正转，0-----反转
//speed: 速度，默认最大取10000
//摩托左右控制：左摆 1_0 右摆 0_1（第一个数是1号电机的dir,第二个数是2号电机的dir）
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

