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


//CPU1��Ҫ����PID���㡢��CPU0�в������жϱ�־λ������Ӧ�Ĵ���
void Cpu1_Init(void)
{
    //��·��������ʼ��
    encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);//������һ
    encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8,TIM4_ENCODER_CH2_P00_9);//��������
    lpf_init();
    Kalman_Init();
    MotorInit();
    AllPID_Init();
//    dl1b_init();//���ģ���ʼ��
    imu660ra_init();//�����ǳ�ʼ��
    wireless_uart_init();
    Servo_Init();
    Set_Servo(Servo);
//�����������Լ��䳬ʱ����ʼ��
//    SpeedCal_Init(&speed1,P15_5,P15_3);
//    SpeedCal_Init(&speed2,P15_4,P15_2);
    //cpu_wait_event_ready();
    key_init(10);

    LedScreen_Init();

#if !(Debug_Mode)
        seekfree_assistant_interface_init (SEEKFREE_ASSISTANT_WIRELESS_UART);
#endif

    pit_ms_init(CCU60_CH0, 5);//PID����     ��ʱ����ʼ��Ҫ�������г�ʼ�����棬��Ȼ�Ῠ��ʱ���жϳ���
}
