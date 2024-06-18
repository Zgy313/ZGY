#include <PID.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdlib.h>
#include <zf_device_dot_matrix_screen.h>
#include <zf_device_key.h>
#include "LedScreen.h"

/*
* ���߶��壺
*                  ------------------------------------
*                  TLD7002����ģ��      ��Ƭ���ܽ�
*                  SR0                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR0_PIN�궨��
*                  SR1                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR1_PIN�궨��
*                  SR2                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR2_PIN�궨��
*                  SR3                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR3_PIN�궨��
*                  SR4                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR4_PIN�궨��
*                  SR5                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR5_PIN�궨��
*                  SR6                 �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SR6_PIN�궨��
*                  SYNC                �鿴zf_device_dot_matrix_screen.h �� DOT_MATRIX_SCREEN_SYNC_PIN�궨��
*                  RX                  �鿴 zf_device_tld7002.h �� TLD7002_UART_RX �궨��
*                  HSLIL               �鿴 zf_device_tld7002.h �� TLD7002_UART_HLSIL �궨��
*                  HSLIH               ����
*                  GPIN0               �鿴 zf_device_tld7002.h �� TLD7002_GPIN0_PIN �궨��
*                  GPIN1               ����
*                  VCC                 8-15V��Դ
*                  3V3                 3.3V��Դ����ģ���ϵ�ͬ����·���磩
*                  GND                 ��Դ��
*/


const char  temp_string[] = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}";

uint32      stime;
uint8       pit_10ms_flag;
uint8       pit_500ms_flag;
uint8       duty_dir = 1;       // ռ�ձȸı�ķ���
uint8       display_mode = 0;   // ��ʾģʽ 0:������4��ģʽ������ʾ  1��������ʾOK ��NG�� A1�� B2  2����ʾ�ַ�666�������к���Ч�� 3����ʾ�ַ�888����������˸Ч��  4��������ʾȡģ�����е��ַ�
uint16      led_duty = 3000;    // ������ģʽʱ ���Ƶ�������

// **************************** �������� ****************************

void display_example1(void);
void display_example2(void);
void display_example3(void);
void display_example4(void);

void pit_10ms_isr(void)
{
    key_scanner();
    stime++;
    pit_10ms_flag = 1;

    if(0 == (stime % 50))
    {
        pit_500ms_flag = 1;
    }

    if(KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        key_clear_state(KEY_1);
        display_mode = 1;
        stime = 0;
    }
    else if(KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        key_clear_state(KEY_2);
        display_mode = 2;
        stime = 0;
    }
    else if(KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        key_clear_state(KEY_3);
        display_mode = 3;
        stime = 0;
    }
    else if(KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        key_clear_state(KEY_4);
        display_mode = 4;
        stime = 0;
    }
}

void display_example1(void)
{
    // ���õ�������
    dot_matrix_screen_set_brightness(5000);

    // ������ʾOK ��NG�� A1�� B2
    if(0 == (stime%200))
    {
        dot_matrix_screen_show_string("OK ");
    }
    else if(50 == (stime%200))
    {
        dot_matrix_screen_show_string("NG ");
    }
    else if(100 == (stime%200))
    {
        dot_matrix_screen_show_string("A1 ");
    }
    else if(150 == (stime%200))
    {
        dot_matrix_screen_show_string("B2 ");
    }
}

void display_example2(void)
{
    // ��ʾ�ַ� 666
    dot_matrix_screen_show_string("666");

    if(1 == pit_10ms_flag)
    {
        pit_10ms_flag = 0;

        if(duty_dir)
        {
            led_duty += 50;
            if(9500 < led_duty)
            {
                duty_dir = 0;
            }
        }
        else
        {
            led_duty -= 50;
            if(1500 > led_duty)
            {
                duty_dir = 1;
            }
        }
        // ���õ�������
        dot_matrix_screen_set_brightness(led_duty);
    }
}

void display_example3(void)
{
    // ��ʾ�ַ� 888
    dot_matrix_screen_show_string("888");

    if(0 == (stime%100))
    {
        // ���õ�������
        dot_matrix_screen_set_brightness(9500);
    }
    else if(50 == (stime%100))
    {
        // ���õ�������
        dot_matrix_screen_set_brightness(1500);
    }
}

void display_example4(void)
{
    static uint8 temp_position = 0;

    if(pit_500ms_flag)
    {
        dot_matrix_screen_set_brightness(5000);

        pit_500ms_flag = 0;

        // ��ʾ�ַ�
        dot_matrix_screen_show_string(&temp_string[temp_position]);
        temp_position++;

        if((temp_position + 3) >= sizeof(temp_string))
        {
            temp_position = 0;
        }
    }
}


void LedScreen_Init(void)
{

//        key_init(10);                   // ��ʼ������������ɨ������������Ϊ10ms
        dot_matrix_screen_init();       // ������Ļ��ʼ��

//        pit_ms_init(CCU60_CH1, 10);     // ���ڶ�ʱ����ʼ��
}

//��ʾĦ�г�״̬(��while�е���)
uint8_t MotorState=0;
uint8_t RunFlag=0;
void Show_MotorState(void)
{
    if(RunFlag==0)
    {
        if(abs(Roll.ActualValue)>=30)
        {
            MotorState=1;
//            RunFlag=1;
        }else if(abs(Roll.ActualValue)<30)
        {
            MotorState=2;
//            RunFlag=1;
        }
    }else if(RunFlag==1)
    {

    }

    switch(MotorState)
    {
        case 0:
            dot_matrix_screen_show_string("ZWZ");
            break;
        case 1:
            dot_matrix_screen_show_string("NG ");
            break;
        case 2:
            dot_matrix_screen_show_string("OK ");
            break;
        case 3:

            break;
        default:
            break;
    }
    // ���õ�������
    dot_matrix_screen_set_brightness(led_duty);
}


