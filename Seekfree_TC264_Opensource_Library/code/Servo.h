#ifndef _Servo_h_
#define _Servo_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"

//舵机初始化
void Servo_Init(void);

//舵机转向
void Set_Servo(uint8_t angle);





#endif
