#ifndef REPAIR_H
#define REPAIR_H

extern uint8 deviation;

extern uint8 sign;
extern uint8 sign_cross;//十字计数
extern uint8 sign_ring;//圆环计数

extern uint8 cross_1;//十字遇
extern uint8 cross_2;//十字出

extern uint8 ring_1;//圆环 进 左下加弧
extern uint8 ring_2;//出圆环 左上点

extern uint8 deviation;//用偏移当中线

void cross_fill(void);
void clear_sign(uint16 *cnt);


#endif /*_REPAIR_H*/


