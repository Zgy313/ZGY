//    gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.001);
#include "lowpass_filter.h"

/******************************************************************************/
LowPassFilter  lpf_current_gyro;
LowPassFilter  lpf_current_gyro_z;
LowPassFilter   lpf_current_mv;
/******************************************************************************/

void lpf_init(void)
{

    lpf_current_gyro.Tf = 0.08f;
    lpf_current_gyro.y_prev = 0;

    lpf_current_gyro_z.Tf = 0.08f;
    lpf_current_gyro_z.y_prev = 0;

    lpf_current_mv.Tf = 0.08f;
    lpf_current_mv.y_prev = 0;

}


/******************************************************************************/
float lpf_operator(LowPassFilter* LPF,float x ,float dt)
{

    float alpha = 0, y = 0;

    alpha = LPF->Tf / (LPF->Tf + dt);
    y = alpha * LPF->y_prev + (1.0f - alpha) * x;
    LPF->y_prev = y;

    return y;
}
/******************************************************************************/




//float acc_ratio = 0.6;      //���ٶȼƱ���
//
//float gyro_ratio = 0.38;    //�����Ǳ���

//float acc_ratio = 0.000244;      //���ٶȼƱ���
//
//float gyro_ratio = 0.06;    //�����Ǳ���
float angle;                //�����ںϺ�ĽǶ�
float acc_ratio = 1.6;      //���ٶȼƱ���
float gyro_ratio = 4.08;    //�����Ǳ���
float dt = 0.005;           //��������
//----------------------------------------------------------------

//  @brief      һ�׻����˲�

//  @param      angle_m     ���ٶȼ�����

//  @param      gyro_m      ����������

//  @return     float       �����ںϺ�ĽǶ�

//----------------------------------------------------------------

float angle_calc(float angle_m, float gyro_m)
{

    float temp_angle;

    float gyro_now;

    float error_angle;

    static float last_angle;

    static uint8 first_angle;

    if(!first_angle)//�ж��Ƿ�Ϊ��һ�����б�����
    {
        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
        first_angle = 1;

        last_angle = angle_m;
    }

    gyro_now = gyro_m * gyro_ratio;

    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
    error_angle = (angle_m - last_angle)*acc_ratio;

    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
    temp_angle = last_angle + (error_angle + gyro_now)*dt;

    //���浱ǰ�Ƕ�ֵ
    last_angle = temp_angle;

    return temp_angle;

}

