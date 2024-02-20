#include <Kalman.h>
#include <lowpass_filter.h>
#include <Motor.h>
#include <PID.h>
#include <Servo.h>
#include <zf_common_clock.h>
#include <zf_device_imu660ra.h>
#include <zf_device_wireless_uart.h>
#include <zf_driver_pit.h>
#include "Cpu1Init.h"



//CPU1主要用来PID运算、对CPU0中产生的中断标志位进行相应的处理
void Cpu1_Init(void)
{
    //首先需要一个10ms的定时中断用来计算PID


    lpf_init();
    Kalman_Init();
    MotorInit();
    AllPID_Init();
//    Servo_Init();
//    dl1b_init();//测距模块初始化
    imu660ra_init();//陀螺仪初始化
    wireless_uart_init();
//    system_delay_init();
//    //两路编码器初始化
//    encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);//编码器一
//    encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8,TIM4_ENCODER_CH2_P00_9);//编码器三
//    seekfree_assistant_init();
//    Servo_Init();
//    Set_Servo(90);
    cpu_wait_event_ready();

//以下是测试内容
//    ips200_init(IPS200_TYPE_SPI);
//    ips200_full(RGB565_WHITE);
//    ips200_show_char(10,10,'a');
//    printf("ok");
//    MotorRun(1,1,0);
//    MotorRun(2,1,800);

    pit_ms_init(CCU60_CH0, 5);//定时器初始化要放在所有初始化后面，不然会卡定时器中断程序
}
