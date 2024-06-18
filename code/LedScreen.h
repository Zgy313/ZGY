#ifndef _LedScreen_h_
#define _LedScreen_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"

extern uint8_t MotorState;
extern uint8_t RunFlag;
//led点阵初始化
void LedScreen_Init(void);

//led显示样例
void display_example1(void);
void display_example2(void);
void display_example3(void);
void display_example4(void);
//用来检测按键状态（样例使用）
void pit_10ms_isr(void);


//显示摩托车状态(在while中调用)
void Show_MotorState(void);

#endif
