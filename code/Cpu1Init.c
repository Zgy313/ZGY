#include <Cpu1Init.h>
#include <Kalman.h>
#include <lowpass_filter.h>
#include <LedScreen.h>
#include <Motor.h>
#include <PID.h>
#include <seekfree_assistant_interface.h>
#include <Servo.h>
#include <zf_device_imu660ra.h>
#include <zf_device_key.h>
#include <zf_device_wireless_uart.h>
#include <zf_driver_encoder.h>
#include <zf_driver_pit.h>


//CPU1主要用来PID运算、对CPU0中产生的中断标志位进行相应的处理
void Cpu1_Init(void)
{
    //两路编码器初始化
    encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);//编码器一
    encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8,TIM4_ENCODER_CH2_P00_9);//编码器三
    lpf_init();
    Kalman_Init();
    MotorInit();
    AllPID_Init();
//    dl1b_init();//测距模块初始化
    imu660ra_init();//陀螺仪初始化
    wireless_uart_init();
    Servo_Init();
    Set_Servo(Servo);
//编码器测速以及其超时检测初始化
//    SpeedCal_Init(&speed1,P15_5,P15_3);
//    SpeedCal_Init(&speed2,P15_4,P15_2);
    //cpu_wait_event_ready();
    key_init(10);

    LedScreen_Init();

#if !(Debug_Mode)
        seekfree_assistant_interface_init (SEEKFREE_ASSISTANT_WIRELESS_UART);
#endif

    pit_ms_init(CCU60_CH0, 5);//PID计算     定时器初始化要放在所有初始化后面，不然会卡定时器中断程序
}
