#ifndef LOSE_H
#define LOSE_H

typedef struct
{
    float P; //����Э����
    float G; //����������
    float Q; //��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    float R; //��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��
    float Output; //�������˲������
}KalmanFilter;


extern uint8 Lost_line_l;//��߶���
extern uint8 Lost_line_r;
extern KalmanFilter kf_distance;
extern float distance;
void  lose_midline(void);

#endif /*_LOSE_H*/
