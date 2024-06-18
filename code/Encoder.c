#include <Encoder.h>
#include <zf_driver_timer.h>

SPEED speed1;
SPEED speed2;


//编码器测速初始化
void SpeedCal_Init(SPEED *speed,gpio_pin_enum A,gpio_pin_enum B)
{
    //参数初始化
    speed->DIR=0;
    speed->ENCODER.Kg=0.02f;
    speed->ENCODER.LastP=0;
    speed->ENCODER.Now_P=0;
    //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    speed->ENCODER.Q=0.0118f;
    speed->ENCODER.R=0.6480f;
    speed->ENCODER.out=0;
    speed->Flag=0;
    speed->PinA=A;
    speed->PinB=B;
    speed->PinAState=0;
    speed->PinBState=0;
    speed->T=0;
    speed->lastspeed=0;
    speed->speed=0;
    //1号电机编码器测速
    //GPIO 外部中断初始化  P15_5  P15_3
    gpio_init(speed->PinA,GPI,0,GPI_PULL_DOWN); //A相
    gpio_init(speed->PinB,GPI,0,GPI_PULL_DOWN); //B相
    exti_init(ERU_CH4_REQ13_P15_5,EXTI_TRIGGER_RISING);
    //2号电机编码器测速 p15_4  p15_2
    exti_init(ERU_CH0_REQ0_P15_4,EXTI_TRIGGER_RISING);
    system_start();         //开始计时


}

//速度计算
void SpeedCal(SPEED *speed)
{
    speed->Flag++;
    switch(speed->Flag)
    {
        case 1:

            speed->PinAState=gpio_get_level(speed->PinA);
            speed->PinBState=gpio_get_level(speed->PinB);
            speed->DIR=(speed->PinAState==speed->PinBState)?(1):(-1);
            speed->M1T=system_getval_ms()/1000.0;

            break;
        case 2:
            speed->T=system_getval_ms()/1000.0-speed->M1T;
            // 后轮直径 D=10cm T法测速：V=(3.1415926*D)/(Z*T)   T为两个脉冲的间隔（单位为s）,V的单位为cm/s
            speed->speed=(Pi*D)/(Z*speed->T);
            speed->speed*=speed->DIR;
            if(speed->speed>speed->lastspeed+20)
            {
                speed->speed=speed->lastspeed+20;
            }else if(speed->speed<speed->lastspeed-20)
            {
                speed->speed=speed->lastspeed-20;
            }
            speed->speed=kalmanFilter(&speed->ENCODER,speed->speed);
            speed->lastspeed=speed->speed;
            speed->Flag=0;
            speed->M1T=0;
            speed->M2T=0;
            speed->T=0;
            speed->timeout=0;
            system_start();//重新启动计时器
            break;
        default :
            speed->Flag=0;
            speed->M1T=0;
            speed->M2T=0;
            speed->T=0;
            system_start();//重新启动计时器
            speed->speed=0;
            speed->timeout=0;
            break;

    }

}

//测速超时检测(放到2ms定时中断中)
void Time_Out_Detect(SPEED *speed)
{

    if(speed->Flag==1)
    {
    speed->timeout++;
    }else if(speed->Flag==0)
    {
        speed->timeout=0;

    }
    if(speed->timeout>=5)//100ms
    {

        speed->Flag=0;
        speed->M1T=0;
        speed->M2T=0;
        speed->T=0;
        system_start();//重新启动计时器
        speed->speed=0;
        speed->timeout=0;

    }

}
