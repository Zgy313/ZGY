#ifndef REPAIR_H
#define REPAIR_H

extern uint8 deviation;

extern uint8 sign;
extern uint8 sign_cross;//ʮ�ּ���
extern uint8 sign_ring;//Բ������

extern uint8 cross_1;//ʮ����
extern uint8 cross_2;//ʮ�ֳ�

extern uint8 ring_1;//Բ�� �� ���¼ӻ�
extern uint8 ring_2;//��Բ�� ���ϵ�

extern uint8 deviation;//��ƫ�Ƶ�����

void cross_fill(void);
void clear_sign(uint16 *cnt);


#endif /*_REPAIR_H*/


