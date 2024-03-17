#ifndef _Servo_h_
#define _Servo_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"
#include <PID.h>


extern float ServoAid;

//�����ʼ��
void Servo_Init(void);

//���ת��
void Set_Servo(float angle);

//�������ƽ��
void Balance_Aid(void);



#endif
