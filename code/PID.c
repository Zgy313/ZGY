#include <Kalman.h>
#include <lose.h>
#include <lowpass_filter.h>
#include <Motor.h>
#include <PID.h>
#include <Quaternion.h>
#include <stdint.h>
#include <stdlib.h>
#include <Servo.h>
#include <zf_device_imu660ra.h>
#include <zf_driver_encoder.h>

#pragma section all "cpu1_dsram"

extern float GAIN;

#define SMIN 700  //��С�ٶ�
#define ZSMAX 2000  //ֱ������ٶ�
//���ҵ��������
#define M1DeadLine  120
#define M2DeadLine  20
//�ٶȹ滮
//0 ֱ�� 1 ��� 2 Բ�� 3 ���µ� 4���µ�
//ǰ���ٶ�
const float SeepVal=-8;
const float Zero=-5.1;//��е��㣨��̬��㣬Ħ�г���ת��ʱ��Ҫ��һ������б�Ƕȣ�
//��ǰ����λ��
uint8_t Pos=0;

PID MotorCtr;
PID Roll;
PID Gx;
PID Yaw;
PID Turn;
int16_t Out1=0,Out2=0;
//u8 PIDCalFlag=0;
u8 tm=0,tr=0,tg=0,ty=0,ts=0;
float M1_Encoder=0,M2_Encoder=0,DM_Encoder=0;
extern float p;
extern float i;
extern float d;
float gyro_lpf=0;//���ٶ�ֵ
float gyro_z_out=0;
extern float gain;
#pragma section all "cpu1_psram"
//������PID��������ʼ��
void AllPID_Init(void)
{
    //��PID�����������޸�

    //ǰ��������ٶȻ�
//    PID_Init(&MotorCtr,50,6.5,32,9000); //�ֵ�
    PID_Init(&MotorCtr,69,14.7,18,9000); //Ч�����õ�

    //ת��PID(Yaw���ٶȻ�������ʽ) �ο����� &Turn,0.074,0,0.007,0,9000

//    POS_PID_Init(&Turn,0.051,0,0.014,0,9000);
    POS_PID_Init(&Turn,0.035,0,0.01,0,9000);


    //(Yaw���ٶȻ�������ʽ) �ο����� 0.032,0,0.018,0,9000

    POS_PID_Init(&Yaw,0.033,0,0.029,0,9000);//(����)

    //�ǶȻ�PID�ο�������λ��ʽ��
    // 2.4,0,0.76,500,9000
    //2.46,0,0.795,500,9000(��)
//    POS_PID_Init(&Roll,6.15,0,0.002,100,9000);//�����磩
    POS_PID_Init(&Roll,5.79,0,0.18,100,9000);//(�����)
//    POS_PID_Init(&Roll,5.84,0,1.35,100,9000);//(����)


    //���ٶȻ��ο�����������ʽ��49,3.4,31,5000

//    PID_Init(&Gx,20,3.1,8,5000);//(����)

    PID_Init(&Gx,49,3.4,31,5000);//�����ʣ�



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

    pid->output+=((pid->error-pid->lastError)*pid->kp)+((pid->error-2*pid->lastError+pid->prev_error)*pid->kd)+pid->error*pid->ki;


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
//    PID->kpout=PID->Kp_pos*PID->ek;
    //�������������
    PID->integral += PID->ek;
    //PID->kiout*=PID->ki_pos;
    //�����޷�
    if(PID->integral > PID->maxIntegral)
        PID->integral=PID->maxIntegral;
//        PID->kiout=0;
    else if(PID->integral < -PID->maxIntegral)
        PID->integral=-PID->maxIntegral;
//        PID->kiout=0;
    //΢�����������
//    PID->kdout=PID->kd_pos*(PID->ek-PID->ek1);
    //PID�����������������

    PID->out=(PID->Kp_pos*PID->ek) + ( PID->kd_pos*(PID->ek-PID->ek1) ) + PID->ki_pos * PID->integral;


//    PID->out=PID->kpout+PID->kiout+PID->kdout;
    //����޷�
    if(PID->out > PID->maxOutput) PID->out=PID->maxOutput;
    else if(PID->out < -PID->maxOutput) PID->out=-PID->maxOutput;
    //��ʷƫ������ֵ
    PID->ek1=PID->ek;
}



//PID���������(��PIT�жϺ����е���)
void PID_Realize(void)
{
//   static uint8_t dir=0;



/*=========================================================================================================*/
    imu660ra_get_acc();
    imu660ra_get_gyro();

//ת����ٶȻ�
    gyro_z_out = lpf_operator(&lpf_current_gyro_z,imu660ra_gyro_transition(imu660ra_gyro_z),0.005);
//�ǶȻ�
    IMU_Update(imu660ra_gyro_transition(imu660ra_gyro_x),imu660ra_gyro_transition(imu660ra_gyro_y),imu660ra_gyro_transition(imu660ra_gyro_z),imu660ra_acc_transition(imu660ra_acc_x),imu660ra_acc_transition(imu660ra_acc_y),imu660ra_acc_transition(imu660ra_acc_z));
    FilterRoll=kalmanFilter(&gyro_z_str,FilterRoll);

//���ٶȻ�
    gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_transition(imu660ra_gyro_x),0.005);


/*=========================================================================================================*/

//    IMU_quaterToEulerianAngles();

    /*==================================�ٶȻ�==================================*/
        //�ٶȻ�PD(����ʽ)
      if(tm==10)//100ms����һ���ٶȻ�
      {
          //������1 ---�Ҳ���      ������2 ---�����
          M1_Encoder=encoder_get_count(TIM2_ENCODER);
          M2_Encoder=encoder_get_count(TIM4_ENCODER);

          DM_Encoder=M2_Encoder-M1_Encoder;
//          printf("%f,%f\n",SeepVal,DM_Encoder);
          encoder_clear_count(TIM2_ENCODER);
          encoder_clear_count(TIM4_ENCODER);
    //      Motor1.ActualValue=kalmanFilter(&MotorFilter1,Motor1.ActualValue);
    //      Motor2.ActualValue=kalmanFilter(&MotorFilter2,Motor2.ActualValue);

//          printf("%d\n",DM_Encoder);
    /*==============================================================================================================*/
          //���������Դ���
//            if(dir==0)
//            {
//                SeepVal+=200;
//                if(SeepVal>=4000)
//                {
//                    dir=1;
//
//                    SeepVal=4000;
//                }
//            }else if(dir==1)
//            {
//                SeepVal-=200;
//                if(SeepVal<=0)
//                {
//                    dir=0;
//                    SeepVal=0;
//                }
//            }


    /*===================================================================================================================*/

    /*============================�ٶȻ�===================================*/
          PID_Calc(&MotorCtr,SeepVal,DM_Encoder);

          tm=0;
      }

  /*==================================ת����ٶȻ�==================================*/



        Yaw.ActualValue=gyro_z_out;



        Yaw.SetValue=0;



      PID_POS(&Yaw);//����ֵΪ0




      /*======================ת��==========================*/
//      Turn.ActualValue=gain;
      Turn.ActualValue=kf_distance.Output;      //����ͷ�������
//      if(Turn.ActualValue>50)
//      {
//          Turn.ActualValue=50;
//      }else if(Turn.ActualValue<-50)
//      {
//          Turn.ActualValue=-50;
//      }
      Turn.SetValue=0;                //���������м�
      PID_POS(&Turn);
//      Balance_Aid();

    //�������룺�ٶȻ����ϣ��ǶȻ��ͽǼ��ٶȻ��Ĵ��������Դ�ʵ�ֲ���Ч��
    // �ǶȻ�PD,λ��ʽ(���뿨�����˲�)

//     if(tr==5)//10ms����һ�νǶȻ�
//     {

      /*==================================�ǶȻ�==================================*/
            //�����ǻ�ȡ���ݵ����������Ῠ����

         Roll.ActualValue = FilterRoll;
//            printf("%f\n",Roll.ActualValue);
//            Roll.ActualValue=Roll.ActualValue;
//            Yaw.out=0;
//         Turn.out=0;
//            Get_DyZero(&Zero,&Roll.ActualValue);
//         Roll.SetValue=Zero;
              Roll.SetValue=Zero+Yaw.out+Turn.out;
            PID_POS(&Roll);
//            Balance_Aid();



         /*==================================���ٶȻ�==================================*/


        PID_Calc(&Gx,Roll.out, gyro_lpf );//����ֵΪ0




//            //��������
//            Out1=MotorCtr.output;
//            Out2=MotorCtr.output;
//            Out2=0;
//printf("%d,%d\n",Out1,Out2);

//        Out1=-SeepVal+gain;
//        Out2=-SeepVal+GAIN;
        Out1=MotorCtr.output-Gx.output;
        Out2=MotorCtr.output+Gx.output;
        Motor_HIP4082Drive(1,Out1>0?1:0,abs(Out1));
        Motor_HIP4082Drive(2,Out2>0?1:0,abs(Out2));
//            PID_Out(1,0-Gx.output);
//            PID_Out(2,0+Gx.output);
//            PID_Out(1,Motor1.out);
//            PID_Out(2,Motor2.out);
            if(abs(Roll.ActualValue)>30)
            {
                Motor_HIP4082Drive(1,0,0);
                Motor_HIP4082Drive(2,0,0);
            }




}


//PID��������
//num:����ı��
//speed:PID������������ɸ�,MAX=10000
//����������
//       ����ʱ  Motor1---467   Motor2---410

void PID_Out(uint8_t num,int16_t speed)
{
    uint8_t dir=0;

    if(speed>=0)
    {
        dir=1;

    }else
    if(speed<0)
    {

        dir=0;
        speed=-speed;


    }
    if(speed>=9000)
    {
        speed=9000;
    }
    switch(num)
    {
        case 1://500HZ 400
            MotorRun(1,dir,speed);
//            printf("1=%d\n",dir);

        break;
        case 2://500HZ 450
            MotorRun(2,dir,speed);
//            printf("2=%d\n",dir);
        break;
        default:
            break;
    }



}

void Get_DyZero(float *StaticZero,float  *Roll )
{
   float offset = *Roll - *StaticZero;
   static float offset_time = 0;
   if (abs(offset) > 0.5f && ++offset_time > 10)
   {
       *StaticZero = *StaticZero + 0.01f * offset;
//       if(*StaticZero<-7)
//       {
//           *StaticZero=-7;
//       }else if(*StaticZero>7)
//       {
//           *StaticZero=7;
//       }
       offset_time = 0;
   }
}
//�����ٶȹ滮
void Speed_Ctr(uint16_t *speed,const uint8_t motorstate)
{
    switch(motorstate)
    {
        case 0://ֱ�� ���� ����
            *speed += 50;
            *speed = *speed>ZSMAX? (ZSMAX):(*speed);
            break;
        case 1://�������
            *speed -= 100;
            *speed = *speed<SMIN? (SMIN):(*speed);
            break;
        default:
            break;
    }
}


#pragma section all restore
