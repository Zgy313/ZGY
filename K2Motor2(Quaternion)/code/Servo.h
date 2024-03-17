#ifndef _Servo_h_
#define _Servo_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"
#include <PID.h>


extern float ServoAid;

//舵机初始化
void Servo_Init(void);

//舵机转向
void Set_Servo(float angle);

//舵机辅助平衡
void Balance_Aid(void);



#endif
