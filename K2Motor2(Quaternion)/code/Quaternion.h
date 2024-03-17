//IMU.h
#ifndef _IMU_H
#define _IMU_H


#include "IFXPORT.h"
#include "zf_common_typedef.h"

extern float FilterPitch;
extern float FilterRoll;
extern float FilterYaw;
/*************************
*��������IMU_Update
*���룺ֱ�Ӵ�����е�λ������������
*************************/
void IMU_Update(float gyrox,float gyroy,float gyroz,float accx,float accy,float accz);

#endif
