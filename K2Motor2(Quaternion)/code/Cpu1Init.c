#include <Kalman.h>
#include <lowpass_filter.h>
#include <Motor.h>
#include <PID.h>
#include <Servo.h>
#include <zf_common_clock.h>
#include <zf_device_imu660ra.h>
#include <zf_device_wireless_uart.h>
#include <zf_driver_delay.h>
#include <zf_driver_encoder.h>
#include <zf_driver_pit.h>
#include "Cpu1Init.h"



//CPU1��Ҫ����PID���㡢��CPU0�в������жϱ�־λ������Ӧ�Ĵ���
void Cpu1_Init(void)
{
    //������Ҫһ��10ms�Ķ�ʱ�ж���������PID
    system_delay_init();
    //��·��������ʼ��
    encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);//������һ
    encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8,TIM4_ENCODER_CH2_P00_9);//��������
    lpf_init();
    Kalman_Init();
    MotorInit();
    AllPID_Init();
//    Servo_Init();
//    dl1b_init();//���ģ���ʼ��
    imu660ra_init();//�����ǳ�ʼ��
    wireless_uart_init();
//    system_delay_init();

//    seekfree_assistant_init();
    Servo_Init();
    Set_Servo(90);
//    system_delay_ms(20);
//�����������Լ��䳬ʱ����ʼ��
//    SpeedCal_Init(&speed1,P15_5,P15_3);
//    SpeedCal_Init(&speed2,P15_4,P15_2);
    cpu_wait_event_ready();

//�����ǲ�������
//    ips200_init(IPS200_TYPE_SPI);
//    ips200_full(RGB565_WHITE);
//    ips200_show_char(10,10,'a');
//    printf("ok");
//    MotorRun(1,1,0);
//    MotorRun(2,1,800);
//    pit_ms_init(CCU60_CH1,20);//���������ٳ�ʱ���

    pit_ms_init(CCU60_CH0, 1);//PID����     ��ʱ����ʼ��Ҫ�������г�ʼ�����棬��Ȼ�Ῠ��ʱ���жϳ���
}
