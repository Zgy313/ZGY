#ifndef LOSE_H
#define LOSE_H

typedef struct
{
    float P; //估算协方差
    float G; //卡尔曼增益
    float Q; //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    float R; //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    float Output; //卡尔曼滤波器输出
}KalmanFilter;


extern uint8 Lost_line_l;//左边丢线
extern uint8 Lost_line_r;
extern KalmanFilter kf_distance;
extern float distance;
void  lose_midline(void);

#endif /*_LOSE_H*/
