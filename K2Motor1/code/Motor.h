#ifndef _Motor_h_
#define _Motor_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"



//Ħ����ˢ�����ʼ������
void MotorInit(void);

//Ħ�е����ˢ�����������
void MotorRun(uint8_t num ,uint8_t dir ,int16_t speed);








#endif
