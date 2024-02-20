#include <Encoder.h>
#include <Ifx_FftF32.h>
#include <Ifx_Types.h>
#include <Kalman.h>
#include <lowpass_filter.h>
#include <Motor.h>
#include <PID.h>
#include <stdint.h>
#include <zf_device_imu660ra.h>

#pragma section all "cpu1_dsram"

KFP MotorFilter1;
KFP MotorFilter2;
PID Motor1;
PID Motor2;
PID Roll;
PID Gx;
PID Yaw;
u16 dynzero=8;//��е��㣨��̬��㣬Ħ�г���ת��ʱ��Ҫ��һ������б�Ƕȣ�  8��
//u8 PIDCalFlag=0;
u8 tm=0,tr=0,tg=0,ty=0;
float gyro_lpf=0;//���ٶ�ֵ
#pragma section all "cpu1_psram"
//������PID��������ʼ��
void AllPID_Init(void)
{
     //��PID�����������޸�
    //�ٶȻ�PI����ʱ���ӣ����⣺���Ϻ�С����ֹʱ�޷�����ƽ�⣩
//    PID_Init(&Motor1,1.48,0.65,17.6,10000);
//    PID_Init(&Motor2,12.4,12.2,31.2,10000);

    //ת��PID(Yaw���ٶȻ�������ʽ) �ο����� p 0.34 i0.001 d 0.083
    POS_PID_Init(&Yaw,0.0094,0.0001,0.008,1000,10000);
    //�ǶȻ�PID�ο�������λ��ʽ�� p 0.362 i 0.0001 d 0.028
    POS_PID_Init(&Roll,0.362,0.0001,0.028,1000,10000);
    //���ٶȻ��ο�����������ʽ�� p 0.81 i 0.152 d 7.8
    PID_Init(&Gx,0.81,0.152,7.89,9000);
//    Motor1.reference=-0;       //cm/s
//    Motor2.reference=-0;
    Yaw.SetValue=0;

    //�������˲� ��ʼ��
    MotorFilter1.Kg=0.02f;
    MotorFilter1.LastP=0;
    MotorFilter1.Now_P=0;
    //��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    //��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��
    MotorFilter1.Q=0.002f;
    MotorFilter1.R=2.2f;
    MotorFilter1.out=0;

    //�������˲� ��ʼ��
    MotorFilter2.Kg=0.02f;
    MotorFilter2.LastP=0;
    MotorFilter2.Now_P=0;
    //��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    //��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��
    MotorFilter2.Q=0.01f;
    MotorFilter2.R=3.2f;
    MotorFilter2.out=0;

}

//���ڳ�ʼ��λ��ʽpid�����ĺ���
void POS_PID_Init(PID *PID,float p,float i,float d,float maxI,float maxOut)
{
    PID->Kp_pos=p;
    PID->ki_pos=i;
    PID->kd_pos=d;
    PID->maxIntegral=maxI;
    PID->maxOutput=maxOut;//MAX=10000��Ĭ�ϣ�
}

//���ڳ�ʼ��pid�����ĺ���
void PID_Init(PID *pid,float p,float i,float d,float maxOut)
{
    pid->kp=p;
    pid->ki=i;
    pid->kd=d;
    pid->maxOutput=maxOut;//MAX=10000��Ĭ�ϣ�
}

//����ʽpid
//����Ϊ(pid�ṹ��,Ŀ��ֵ,����ֵ)������������pid�ṹ���output��Ա��
void PID_Calc(PID *pid,float reference,float feedback)
{
//    float dout=0;
//    float pout=0;
    cfloat32 in;
    cfloat32 out;
    //��������
    pid->error=0;
    pid->error=reference-feedback;//������error
//    //����΢��
//     dout=(pid->error-2*pid->lastError+pid->prev_error)*pid->kd;
//    //�������
//     pout=(pid->error-pid->lastError)*pid->kp;
    //�������
//    pid->integral=pid->error*pid->ki;

    //�������
//    pid->output=pout+dout+pid->integral;
    /*====================�����ǲ���FFT����PID========================*/
    in.real=((pid->error-pid->lastError)*pid->kp)+((pid->error-2*pid->lastError+pid->prev_error)*pid->kd)+pid->error*pid->ki;
    in.imag = 0.0;
    Ifx_FftF32_radix2(&out, &in, 1);//��һ������Ϊ���������ַ���ڶ���Ϊ���������ַ��������Ϊ���������������һ�㶼��1����������⣩
    pid->output+=out.real;
    /*=============================================================*/
    pid->prev_error=pid->lastError;//�����ϴε�error��ΪԤ��ֵ
    pid->lastError=pid->error;//����error������

    //����޷�
    if(pid->output > pid->maxOutput) pid->output=pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output=-pid->maxOutput;
}
//λ��ʽpid
void PID_POS(PID *PID)
{

    //��ǰƫ��
    PID->ek=PID->SetValue-PID->ActualValue;
    //�������������
    PID->kpout=PID->Kp_pos*PID->ek;
    //�������������
    PID->kiout+=PID->ek;
    PID->kiout*=PID->ki_pos;
    //�����޷�
    if(PID->kiout > PID->maxIntegral)
        PID->kiout=0;
    else if(PID->kiout < -PID->maxIntegral) PID->kiout=0;
    //΢�����������
    PID->kdout=PID->kd_pos*(PID->ek-PID->ek1);
    //PID�����������������
    PID->out=PID->kpout+PID->kiout+PID->kdout;
    //����޷�
    if(PID->out > PID->maxOutput) PID->out=PID->maxOutput;
    else if(PID->out < -PID->maxOutput) PID->out=PID->maxOutput;
    //��ʷƫ������ֵ
    PID->ek1=PID->ek;
}



//PID���������(��PIT�жϺ����е���)
void PID_Realize(void)
{
//   static uint8_t dir=0;

    gyro_lpf=0;
    imu660ra_get_acc();
    imu660ra_get_gyro();
    //����ס����Ϊ��ʱ����ʼ����������ǰ�棬ȫ����ʼ����δ��ɱ�������жϣ����³��������ж���,û��ʼ������Ҳ�Ῠ
    //ת����ٶȻ�
    if(ty==3)
    {
//        gyro_z_out = lpf_operator(&lpf_current_gyro,imu660ra_gyro_transition(imu660ra_gyro_z),0.005)-1.533;
        Yaw.ActualValue+=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_z))-0.11;//��ȥ��Ư
//        printf("%f\n",Yaw.ActualValue);
        if((Yaw.ActualValue<0.01)&&(Yaw.ActualValue>-0.01)) Yaw.ActualValue=0;
        PID_POS(&Yaw);//����ֵΪ0
//        printf("yaw=%f\n",Yaw.out);
        ty=0;
    }


    //�������룺�ٶȻ����ϣ��ǶȻ��ͽǼ��ٶȻ��Ĵ��������Դ�ʵ�ֲ���Ч��
    // �ǶȻ�PD,λ��ʽ(���뿨�����˲�)
     if(tr==2)//10ms����һ�νǶȻ�
     {
         //�����ǻ�ȡ���ݵ����������Ῠ����

         Roll.ActualValue+=Kalman_getAngle(0.01,imu660ra_acc_transition(imu660ra_acc_y),imu660ra_gyro_transition(imu660ra_gyro_x));
//         printf("%f\n",Roll.ActualValue);
         Roll.SetValue=dynzero+Yaw.out;
//         Roll.SetValue=dynzero+0;
         PID_POS(&Roll);
//         printf("roll=%f\n",Roll.out);
         tr=0;
     }
     //�Ǽ��ٶȻ�PI������ʽ����ͨ�˲���
      if(tg==1)//5ms����һ�νǼ��ٶȻ�
      {
//          gyro_lpf=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_z));//��ȥ��Ư
          gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.005)-1.29;
//          gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.005);
          PID_Calc(&Gx,Roll.out, gyro_lpf);//����ֵΪ0
//          printf("%f\n",gyro_lpf);
//          printf("Gx=%f\n",Gx.output);
          tg=0;

      }

      //�ٶȻ�PI(����ʽ)
//    if(tm==6)//30ms����һ���ٶȻ�
//    {


        /*================================================================*/
//         Motor1.feedback=speed1.speed;
//         Motor2.feedback=-speed2.speed;//  ���1�͵��2�ǶԳƵģ�������ֵ�෴
         /*================================================================*/
/*==============================================================================================================*/
        //���������Դ���
//        if(dir==0)
//        {
//            Motor1.output+=200;
//            if(Motor1.output>=4000)
//            {
//            dir=1;
//
//            Motor1.output=4000;
//            }
//        }else if(dir==1)
//        {
//            Motor1.output-=200;
//            if(Motor1.output<=0)
//            {
//            dir=0;
//            Motor1.output=0;
//            }
//        }
//        Motor1.output=2000;
//        Motor2.output=0;
//        printf("%f\n",Motor1.feedback);
/*===================================================================================================================*/

/*============================�ٶȻ�===================================*/
//        PID_Calc(&Motor1, Motor1.reference, Motor1.feedback);
//        PID_Calc(&Motor2, Motor2.reference, Motor2.feedback);
//        Motor1.output=kalmanFilter(&MotorFilter1,Motor1.output);
//        Motor2.output=kalmanFilter(&MotorFilter2,Motor2.output);
/*====================================================================*/
//        printf("%f,%f\n",Motor2.reference,Motor2.feedback);
//        tm=0;
//    }


        //���(�Ῠ����)
//        PID_Out(1,Motor1.reference*20+Motor1.output-Gx.output);
//        PID_Out(2,Motor2.reference*20+Motor2.output+Gx.output);

//    printf("motor1out=%f\n",Motor1.output);
            PID_Out(1,0-Gx.output);
            PID_Out(2,0+Gx.output);

}


//PID��������
//num:����ı��
//speed:PID������������ɸ�,MAX=10000
//������������û��ʱ Motor1---700   Motor2---1000
//       ����ʱ  Motor1---650   Motor2---800
void PID_Out(uint8_t num,int16_t speed)
{
    uint8_t dir=0;

    if(speed>=0)
    {
        dir=1;

    }else if(speed<0)
    {

        dir=0;
        speed=-speed;


    }
    if(speed>=10000)
    {
        speed=9999;
    }
    switch(num)
    {
        case 1:
            MotorRun(1,dir,speed+650);

        break;
        case 2:
            MotorRun(2,dir,speed+800);
        break;
    }



}





#pragma section all restore
