#ifndef _ENCODER_H_
#define _ENCODER_H_


#include "../libraries/zf_driver/zf_driver_exti.h"
#include "../libraries/zf_driver/zf_driver_gpio.h"
#include "Kalman.h"

//K2车模后轮 光栅齿轮一圈30脉冲  减速比 16：50（光栅：车轮） 车轮一转产生约Z=94脉冲
// 后轮直径 D=10cm T法测速：V=(3.1415926*D)/(Z*T)   T为两个脉冲的间隔（单位为s）,V的单位为cm/s

#define D 10.0f    //直径
#define Pi  3.1415926   //圆周率
#define Z  94    //车轮一转产生约Z=94脉冲



typedef struct
{
        int8_t DIR;             //转动方向
        uint8_t Flag;           //测速标志位(1---开始测速，2---测速完成)
        float T;                //两个脉冲的间隔时间
        float M1T;              //摩托1号电机编码器测速计时起始时间
        float M2T;              //摩托2号电机编码器测速计时起始时间
        uint8_t timeout;          //超时检测
        float speed;            //转速
        float lastspeed;        //上次转速
        KFP ENCODER;            //用来初始化卡尔曼滤波
        gpio_pin_enum PinA;     //A相引脚
        gpio_pin_enum PinB;     //B相引脚
        uint8_t PinAState;      //A相引脚状态
        uint8_t PinBState;      //B相引脚状态

}SPEED;


extern SPEED speed1;
extern SPEED speed2;

//编码器测速初始化
void SpeedCal_Init(SPEED *speed,gpio_pin_enum A,gpio_pin_enum B);

//速度计算
void SpeedCal(SPEED *speed);

//测速超时检测(放到2ms定时中断中)
void Time_Out_Detect(SPEED *speed);

#endif
