#ifndef _LedScreen_h_
#define _LedScreen_h_

#include "IFXPORT.h"
#include "zf_common_typedef.h"

extern uint8_t MotorState;
extern uint8_t RunFlag;
//led�����ʼ��
void LedScreen_Init(void);

//led��ʾ����
void display_example1(void);
void display_example2(void);
void display_example3(void);
void display_example4(void);
//������ⰴ��״̬������ʹ�ã�
void pit_10ms_isr(void);


//��ʾĦ�г�״̬(��while�е���)
void Show_MotorState(void);

#endif
