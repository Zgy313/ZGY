#ifndef _Motor_h_
#define _Motor_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"



//摩托有刷电机初始化函数
void MotorInit(void);

//摩托电机有刷电机驱动函数
void MotorRun(uint8_t num ,uint8_t dir ,int16_t speed);


//HIP4082驱动芯片驱动函数，与L298N驱动方式一样
void Motor_HIP4082Drive(uint8_t num ,uint8_t dir ,int16_t speed);





#endif
