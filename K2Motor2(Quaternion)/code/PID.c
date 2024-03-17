#include <Ifx_FftF32.h>
#include <Ifx_Types.h>
#include <Kalman.h>
#include <lowpass_filter.h>
#include <Motor.h>
#include <PID.h>
#include <Quaternion.h>
#include <stdint.h>
#include <Servo.h>
#include <zf_device_imu660ra.h>

#pragma section all "cpu1_dsram"

KFP MotorFilter1;
KFP MotorFilter2;
PID Motor1;
PID Motor2;
PID MotorFW;
PID Roll;
PID Gx;
PID Yaw;

float dynzero=-1.0;//��е��㣨��̬��㣬Ħ�г���ת��ʱ��Ҫ��һ������б�Ƕȣ�  -0.65
//u8 PIDCalFlag=0;
u8 tm=0,tr=0,tg=0,ty=0,ts=0;
extern float p;
extern float i;
extern float d;
float gyro_lpf=0;//���ٶ�ֵ
float gyro_z_out=0;
#pragma section all "cpu1_psram"
//������PID��������ʼ��
void AllPID_Init(void)
{
    //��PID�����������޸�
    //�ٶȻ�PD����ʱ���ӣ����⣺���Ϻ�С����ֹʱ�޷�����ƽ�⣩
    //�����ٶȻ�PI
//        POS_PID_Init(&Motor1,667.9,132.5,222.9,100,10000);
//        POS_PID_Init(&Motor2,973.5,193.5,291.5,100,10000);
    //С���ٶȻ�PD
    //0.26 0.0008 0.228
    POS_PID_Init(&Motor1,0.26,0.2,0.31,0.6,90);
//    POS_PID_Init(&Motor1,0.26,0.0008,0.228,1,90);
//    POS_PID_Init(&Motor1,0.14,0.003,0.47,15,90);
//    POS_PID_Init(&Motor1,0.44,0.04,0.4,10,90);
    //ת��PID(Yaw���ٶȻ�������ʽ) �ο����� 0.1 0.0018 0.03 100 9000
    //    POS_PID_Init(&Yaw,0.1,0.0048,0.03,100,9000);
    //�ǶȻ�PID�ο�������λ��ʽ�� p 9.6 i 0.0001 d 1.5
    //    POS_PID_Init(&Roll,20.6,0,1.58,1000,9000);
    // 25.6 0.34 2.92
    POS_PID_Init(&Roll,26.26,0.26,2.962,20,9000);
    //���ٶȻ��ο�����������ʽ�� p 0.81 i 0.152 d 7.8
    PID_Init(&Gx,1.2,0.035,19.3,8000);


    Motor1.SetValue=0;       //cm/s    30-40����
//    Motor2.SetValue=0;
//    Yaw.SetValue=0;

    //�������˲� ��ʼ��
    MotorFilter1.Kg=0.02f;
    MotorFilter1.LastP=0;
    MotorFilter1.Now_P=0;
    //��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    //��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��
    MotorFilter1.Q=0.001f;
    MotorFilter1.R=0.543f;
    MotorFilter1.out=0;

    //�������˲� ��ʼ��
    MotorFilter2.Kg=0.02f;
    MotorFilter2.LastP=0;
    MotorFilter2.Now_P=0;
    //��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    //��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��
    MotorFilter2.Q=0.001f;
    MotorFilter2.R=0.543f;
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
//    cfloat32 in;
//    cfloat32 out;

    //��ǰƫ��
    PID->ek=PID->SetValue-PID->ActualValue;
    //�������������
    PID->kpout=PID->Kp_pos*PID->ek;
    //�������������
    PID->kiout+=PID->ek;
    PID->kiout*=PID->ki_pos;
    //�����޷�
    if(PID->kiout > PID->maxIntegral)
//        PID->kiout=PID->maxIntegral;
        PID->kiout=0;
    else if(PID->kiout < -PID->maxIntegral)
//        PID->kiout=-PID->maxIntegral;
        PID->kiout=0;
    //΢�����������
    PID->kdout=PID->kd_pos*(PID->ek-PID->ek1);
    //PID�����������������
    /*===================�������======================*/
//    in.real=(PID->Kp_pos*PID->ek)+( PID->kd_pos*(PID->ek-PID->ek1) )+PID->kiout;
//    in.imag=0;
//    Ifx_FftF32_radix2(&out, &in, 1);//��һ������Ϊ���������ַ���ڶ���Ϊ���������ַ��������Ϊ���������������һ�㶼��1����������⣩
//    PID->out=out.real;
    /*===============================================*/
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

    Motor1.ActualValue=(float)encoder_get_count(TIM2_ENCODER);
    Motor1.ActualValue=kalmanFilter(&MotorFilter1,Motor1.ActualValue);
    /*=================������������ٶȻ�ȡ����======================*/
//    Motor2.ActualValue=-(float)encoder_get_count(TIM4_ENCODER);
    Motor2.ActualValue=(float)encoder_get_count(TIM4_ENCODER);
    Motor2.ActualValue=kalmanFilter(&MotorFilter2,Motor2.ActualValue);

    //�ٶȻ�PD(����ʽ)
  if(tm==100)//100ms����һ���ٶȻ�
  {


      /*================================================================*/
//         Motor1.feedback=speed1.speed;
//         Motor2.feedback=-speed2.speed;//  ���1�͵��2�ǶԳƵģ�������ֵ�෴
       /*================================================================*/

      // 386 6 102
//      Motor1.ActualValue=((Motor1.ActualValue/94.0f)*(3.1415926f*10.0f))*1000.0f/(float)tm;   //  cm/s
//      Motor2.ActualValue=((Motor2.ActualValue/94.0f)*(3.1415926f*10.0f))*1000.0f/(float)tm;   //  cm/s
      encoder_clear_count(TIM2_ENCODER);
      encoder_clear_count(TIM4_ENCODER);
//      Motor1.ActualValue=kalmanFilter(&MotorFilter1,Motor1.ActualValue);
//      Motor2.ActualValue=kalmanFilter(&MotorFilter2,Motor2.ActualValue);
      Motor1.ActualValue=Motor1.ActualValue+Motor2.ActualValue;
//      Motor1.ActualValue = lpf_operator(&lpf_current_gyro,Motor1.ActualValue,0.1);
                                         //0.7:-0.7
      Motor1.SetValue=Roll.ActualValue>0?0.74:-0.74;
//      if(Motor1.ActualValue==0 )Motor1.SetValue=0;
/*==============================================================================================================*/
      //���������Դ���
//        if(dir==0)
//        {
//            Motor2.out+=200;
//            if(Motor2.out>=4000)
//            {
//                dir=1;
//
//                Motor2.out=4000;
//            }
//        }else if(dir==1)
//        {
//            Motor2.out-=200;
//            if(Motor2.out<=0)
//            {
//                dir=0;
//                Motor2.out=0;
//            }
//        }
//        Motor1.output=2000;
//        Motor2.output=2000;
//        seekfree_assistant_oscilloscope_data.channel_num=2;
////        seekfree_assistant_oscilloscope_data.data[0]=Motor1.ActualValue;
//        seekfree_assistant_oscilloscope_data.data[1]=Motor2.ActualValue;
//        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//        seekfree_assistant_oscilloscope_data.channel_num=2;
//        seekfree_assistant_oscilloscope_data.data[0]=Motor1.SetValue;
//        seekfree_assistant_oscilloscope_data.data[1]=Motor1.ActualValue;
//        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//        printf("%f,%f\n",Motor1.ActualValue,Motor2.ActualValue);

/*===================================================================================================================*/

/*============================�ٶȻ�===================================*/
      PID_POS(&Motor1);
//      PID_POS(&Motor2);

/*====================================================================*/
//        printf("%f,%f\n",Motor2.reference,Motor2.feedback);
      tm=0;
  }
    //����ס����Ϊ��ʱ����ʼ����������ǰ�棬ȫ����ʼ����δ��ɱ�������жϣ����³��������ж���,û��ʼ������Ҳ�Ῠ
    //ת����ٶȻ�
//    if(ty==10)
//    {
//          gyro_z_out=0;
//          gyro_z_out = lpf_operator(&lpf_current_gyro,imu660ra_gyro_transition(imu660ra_gyro_z),0.02)-0.027f;
////        Yaw.ActualValue=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_z));//��ȥ��Ư
//          Yaw.ActualValue=gyro_z_out;
////        if((Yaw.ActualValue<0.01)&&(Yaw.ActualValue>-0.01)) Yaw.ActualValue=0;
////        printf("%f\n",Yaw.ActualValue);
//        PID_POS(&Yaw);//����ֵΪ0
////        printf("yaw=%f\n",Yaw.out);
////        seekfree_assistant_oscilloscope_data.channel_num=2;
////        seekfree_assistant_oscilloscope_data.data[0]=Yaw.SetValue;
////        seekfree_assistant_oscilloscope_data.data[1]=Yaw.ActualValue;
////        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//        ty=0;
//    }


    //�������룺�ٶȻ����ϣ��ǶȻ��ͽǼ��ٶȻ��Ĵ��������Դ�ʵ�ֲ���Ч��
    // �ǶȻ�PD,λ��ʽ(���뿨�����˲�)
     if(tr==5)//5ms����һ�νǶȻ�
     {
            //�����ǻ�ȡ���ݵ����������Ῠ����
            IMU_Update(imu660ra_gyro_transition(imu660ra_gyro_x),imu660ra_gyro_transition(imu660ra_gyro_y),imu660ra_gyro_transition(imu660ra_gyro_z),imu660ra_acc_transition(imu660ra_acc_x),imu660ra_acc_transition(imu660ra_acc_y),imu660ra_acc_transition(imu660ra_acc_z));
            Roll.ActualValue=FilterRoll-0.003f;
            Roll.ActualValue=(int)(Roll.ActualValue/1);
            //         Roll.ActualValue=kalmanFilter(&gyro_z_str,Roll.ActualValue);
            //         printf("%f\n",Roll.ActualValue);
//            Yaw.out=0;
//            Motor1.out=0;

              Roll.SetValue=dynzero+Motor1.out;
//            Roll.SetValue=dynzero;
            //         Roll.SetValue=dynzero+0;
            PID_POS(&Roll);
            //         Balance_Aid(&Roll);
            //         printf("roll=%f\n",Roll.out);
//            seekfree_assistant_oscilloscope_data.channel_num=2;
//            seekfree_assistant_oscilloscope_data.data[0]=Roll.SetValue;
//            seekfree_assistant_oscilloscope_data.data[1]=Roll.ActualValue;
//            seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

            tr=0;
            ts=0;

     }
     //�Ǽ��ٶȻ�PI������ʽ����ͨ�˲���
      if(tg==1)//1ms����һ�νǼ��ٶȻ�
      {
            //          gyro_lpf=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_x));//��ȥ��Ư
            gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.001)+0.5;
            //          gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.005);
            //          Roll.out=0;//����ֵ0
            PID_Calc(&Gx,Roll.out, gyro_lpf);//����ֵΪ0
            //          printf("%f\n",gyro_lpf);
            //          printf("Gx=%f\n",Gx.output);
            //          seekfree_assistant_oscilloscope_data.channel_num=2;
            //          seekfree_assistant_oscilloscope_data.data[0]=0;
            //          seekfree_assistant_oscilloscope_data.data[1]=gyro_lpf;
            //          seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
            tg=0;


            //          printf("%f,%f",Roll.out,gyro_lpf);
      }


//    printf("ok!\n");
//      seekfree_assistant_oscilloscope_data.data[2]=Motor1.out;
//      seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//      printf("%f\n",Motor1.out);
        //���(�Ῠ����)
//        PID_Out(1,2000);
//        PID_Out(2,2000);

//    printf("motor1out=%f\n",Motor1.output);
            PID_Out(1,-0-Gx.output);
            PID_Out(2,-0+Gx.output);

        //�������ƽ��
        if(ts==0)
        {
//            imu660ra_get_acc();
//            imu660ra_get_gyro();
//            IMU_Update(imu660ra_gyro_transition(imu660ra_gyro_x),imu660ra_gyro_transition(imu660ra_gyro_y),imu660ra_gyro_transition(imu660ra_gyro_z),imu660ra_acc_transition(imu660ra_acc_x),imu660ra_acc_transition(imu660ra_acc_y),imu660ra_acc_transition(imu660ra_acc_z));
//            ServoAid=FilterRoll+0.81f;
            Balance_Aid();

        }

}


//PID��������
//num:����ı��
//speed:PID������������ɸ�,MAX=10000
//������������û��ʱ Motor1---700   Motor2---1000
//       ����ʱ  Motor1---650   Motor2---800
//                      1200           700
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
            MotorRun(1,dir,speed+790);

        break;
        case 2:
            MotorRun(2,dir,speed+550);
        break;
    }



}





#pragma section all restore
