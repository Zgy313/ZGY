#include <PID.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdlib.h>
#include <zf_device_dot_matrix_screen.h>
#include <zf_device_key.h>
#include "LedScreen.h"

/*
* 接线定义：
*                  ------------------------------------
*                  TLD7002驱动模块      单片机管脚
*                  SR0                 查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SR0_PIN宏定义
*                  SR1                 查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SR1_PIN宏定义
*                  SR2                 查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SR2_PIN宏定义
*                  SR3                 查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SR3_PIN宏定义
*                  SR4                 查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SR4_PIN宏定义
*                  SR5                 查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SR5_PIN宏定义
*                  SR6                 查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SR6_PIN宏定义
*                  SYNC                查看zf_device_dot_matrix_screen.h 中 DOT_MATRIX_SCREEN_SYNC_PIN宏定义
*                  RX                  查看 zf_device_tld7002.h 中 TLD7002_UART_RX 宏定义
*                  HSLIL               查看 zf_device_tld7002.h 中 TLD7002_UART_HLSIL 宏定义
*                  HSLIH               悬空
*                  GPIN0               查看 zf_device_tld7002.h 中 TLD7002_GPIN0_PIN 宏定义
*                  GPIN1               悬空
*                  VCC                 8-15V电源
*                  3V3                 3.3V电源（给模块上的同步电路供电）
*                  GND                 电源地
*/


const char  temp_string[] = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}";

uint32      stime;
uint8       pit_10ms_flag;
uint8       pit_500ms_flag;
uint8       duty_dir = 1;       // 占空比改变的方向
uint8       display_mode = 0;   // 显示模式 0:轮流将4个模式进行显示  1：依次显示OK 、NG、 A1、 B2  2：显示字符666，并带有呼吸效果 3：显示字符888，并带有闪烁效果  4：依次显示取模中所有的字符
uint16      led_duty = 3000;    // 呼吸灯模式时 控制点阵亮度

// **************************** 代码区域 ****************************

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
    // 设置点阵亮度
    dot_matrix_screen_set_brightness(5000);

    // 依次显示OK 、NG、 A1、 B2
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
    // 显示字符 666
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
        // 设置点阵亮度
        dot_matrix_screen_set_brightness(led_duty);
    }
}

void display_example3(void)
{
    // 显示字符 888
    dot_matrix_screen_show_string("888");

    if(0 == (stime%100))
    {
        // 设置点阵亮度
        dot_matrix_screen_set_brightness(9500);
    }
    else if(50 == (stime%100))
    {
        // 设置点阵亮度
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

        // 显示字符
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

//        key_init(10);                   // 初始化按键，按键扫描程序调用周期为10ms
        dot_matrix_screen_init();       // 点阵屏幕初始化

//        pit_ms_init(CCU60_CH1, 10);     // 周期定时器初始化
}

//显示摩托车状态(在while中调用)
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
    // 设置点阵亮度
    dot_matrix_screen_set_brightness(led_duty);
}


