#ifndef _CPU1INIT_h_
#define _CPU1INIT_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"

//调试方式 0----无线串口 1----wifi图传
#define Debug_Mode (0)
#define Servo 91.0f//舵机初始化角度

//CPU1主要用来PID运算、对CPU0中产生的中断标志位进行相应的处理
void Cpu1_Init(void);






#endif
