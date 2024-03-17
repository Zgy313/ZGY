#ifndef _PID_H_
#define _PID_H_

#include "IFXPORT.h"
#include "zf_common_typedef.h"


//首先定义PID结构体用于存放一个PID的数据
typedef struct
{
    //以下是增量式pid
    float kp,ki,kd;//三个系数
    float error,lastError;//误差、上次误差
    float integral,maxIntegral;//积分、积分限幅
    float output,maxOutput;//输出、输出限幅
    float reference,feedback;//目标值、实际值
    float prev_error;
    //以下是位置式pid
    float SetValue;       //设定值
    float ActualValue;  //实际值
    float Kp_pos;       //比例系数
    float ki_pos;      //积分系数
    float kd_pos;      //微分系数
    float T;       //采样周期
    float kpout;   //比例输出控制量
    float kiout;   //积分输出控制量
    float kdout;   //微分输出控制量
    float ek;     //当前偏差
    float ek1;     //上一次偏差
    float ek2;     //上上次偏差
    float out;    //输出控制量
}PID;



extern PID Motor1;
extern PID Motor2;
extern PID Roll;
extern PID Yaw;
extern PID Gx;



//extern u8 PIDCalFlag;
extern u8 tm;
extern u8 tr;
extern u8 tg;
extern u8 ty;
extern u8 ts;
extern u16 roll_kalman;
//将所有PID参数都初始化
void AllPID_Init(void);
//增量式PID参数初始化
void PID_Init(PID *pid,float p,float i,float d,float maxOut);
//位置式pid初始化
void POS_PID_Init(PID *PID,float p,float i,float d,float maxI,float maxOut);
//增量式PID计算函数
void PID_Calc(PID *pid,float reference,float feedback);
//位置式pid
void PID_POS(PID *pid);
//PID总输出函数
void PID_Realize(void);
//PID输出函数
void PID_Out(uint8_t num,int16_t speed);




#endif
