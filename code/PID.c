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

#define SMIN 700  //最小速度
#define ZSMAX 2000  //直行最大速度
//左右电机的死区
#define M1DeadLine  120
#define M2DeadLine  20
//速度规划
//0 直行 1 弯道 2 圆环 3 上坡道 4下坡道
//前进速度
const float SeepVal=-8;
const float Zero=-5.1;//机械零点（动态零点，摩托车在转弯时需要有一定的倾斜角度）
//当前车的位置
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
float gyro_lpf=0;//角速度值
float gyro_z_out=0;
extern float gain;
#pragma section all "cpu1_psram"
//将所有PID参数都初始化
void AllPID_Init(void)
{
    //各PID参数在这里修改

    //前进方向的速度环
//    PID_Init(&MotorCtr,50,6.5,32,9000); //粗调
    PID_Init(&MotorCtr,69,14.7,18,9000); //效果更好点

    //转向环PID(Yaw角速度环，增量式) 参考参数 &Turn,0.074,0,0.007,0,9000

//    POS_PID_Init(&Turn,0.051,0,0.014,0,9000);
    POS_PID_Init(&Turn,0.035,0,0.01,0,9000);


    //(Yaw角速度环，增量式) 参考参数 0.032,0,0.018,0,9000

    POS_PID_Init(&Yaw,0.033,0,0.029,0,9000);//(合适)

    //角度环PID参考参数（位置式）
    // 2.4,0,0.76,500,9000
    //2.46,0,0.795,500,9000(抖)
//    POS_PID_Init(&Roll,6.15,0,0.002,100,9000);//（满电）
    POS_PID_Init(&Roll,5.79,0,0.18,100,9000);//(最合适)
//    POS_PID_Init(&Roll,5.84,0,1.35,100,9000);//(合适)


    //角速度环参考参数（增量式）49,3.4,31,5000

//    PID_Init(&Gx,20,3.1,8,5000);//(满电)

    PID_Init(&Gx,49,3.4,31,5000);//（合适）



}

//用于初始化位置式pid参数的函数
void POS_PID_Init(PID *PID,float p,float i,float d,float maxI,float maxOut)
{
    PID->Kp_pos=p;
    PID->ki_pos=i;
    PID->kd_pos=d;
    PID->maxIntegral=maxI;
    PID->maxOutput=maxOut;//MAX=10000（默认）
}

//用于初始化pid参数的函数
void PID_Init(PID *pid,float p,float i,float d,float maxOut)
{
    pid->kp=p;
    pid->ki=i;
    pid->kd=d;
    pid->maxOutput=maxOut;//MAX=10000（默认）
}

//增量式pid
//参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid,float reference,float feedback)
{
//    float dout=0;
//    float pout=0;

    //更新数据
    pid->error=0;
    pid->error=reference-feedback;//计算新error
//    //计算微分
//     dout=(pid->error-2*pid->lastError+pid->prev_error)*pid->kd;
//    //计算比例
//     pout=(pid->error-pid->lastError)*pid->kp;
    //计算积分
//    pid->integral=pid->error*pid->ki;

    //计算输出
//    pid->output=pout+dout+pid->integral;

    pid->output+=((pid->error-pid->lastError)*pid->kp)+((pid->error-2*pid->lastError+pid->prev_error)*pid->kd)+pid->error*pid->ki;


    pid->prev_error=pid->lastError;//将上上次的error作为预测值
    pid->lastError=pid->error;//将旧error存起来

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output=pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output=-pid->maxOutput;
}

//位置式pid
void PID_POS(PID *PID)
{

    //当前偏差
    PID->ek=PID->SetValue-PID->ActualValue;
    //比例输出控制量
//    PID->kpout=PID->Kp_pos*PID->ek;
    //积分输出控制量
    PID->integral += PID->ek;
    //PID->kiout*=PID->ki_pos;
    //积分限幅
    if(PID->integral > PID->maxIntegral)
        PID->integral=PID->maxIntegral;
//        PID->kiout=0;
    else if(PID->integral < -PID->maxIntegral)
        PID->integral=-PID->maxIntegral;
//        PID->kiout=0;
    //微分输出控制量
//    PID->kdout=PID->kd_pos*(PID->ek-PID->ek1);
    //PID控制器的输出控制量

    PID->out=(PID->Kp_pos*PID->ek) + ( PID->kd_pos*(PID->ek-PID->ek1) ) + PID->ki_pos * PID->integral;


//    PID->out=PID->kpout+PID->kiout+PID->kdout;
    //输出限幅
    if(PID->out > PID->maxOutput) PID->out=PID->maxOutput;
    else if(PID->out < -PID->maxOutput) PID->out=-PID->maxOutput;
    //历史偏差量赋值
    PID->ek1=PID->ek;
}



//PID总输出函数(在PIT中断函数中调用)
void PID_Realize(void)
{
//   static uint8_t dir=0;



/*=========================================================================================================*/
    imu660ra_get_acc();
    imu660ra_get_gyro();

//转向角速度环
    gyro_z_out = lpf_operator(&lpf_current_gyro_z,imu660ra_gyro_transition(imu660ra_gyro_z),0.005);
//角度环
    IMU_Update(imu660ra_gyro_transition(imu660ra_gyro_x),imu660ra_gyro_transition(imu660ra_gyro_y),imu660ra_gyro_transition(imu660ra_gyro_z),imu660ra_acc_transition(imu660ra_acc_x),imu660ra_acc_transition(imu660ra_acc_y),imu660ra_acc_transition(imu660ra_acc_z));
    FilterRoll=kalmanFilter(&gyro_z_str,FilterRoll);

//角速度环
    gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_transition(imu660ra_gyro_x),0.005);


/*=========================================================================================================*/

//    IMU_quaterToEulerianAngles();

    /*==================================速度环==================================*/
        //速度环PD(增量式)
      if(tm==10)//100ms计算一次速度环
      {
          //编码器1 ---右侧电机      编码器2 ---左侧电机
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
          //编码器测试代码
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

    /*============================速度环===================================*/
          PID_Calc(&MotorCtr,SeepVal,DM_Encoder);

          tm=0;
      }

  /*==================================转向角速度环==================================*/



        Yaw.ActualValue=gyro_z_out;



        Yaw.SetValue=0;



      PID_POS(&Yaw);//期望值为0




      /*======================转向环==========================*/
//      Turn.ActualValue=gain;
      Turn.ActualValue=kf_distance.Output;      //摄像头返回误差
//      if(Turn.ActualValue>50)
//      {
//          Turn.ActualValue=50;
//      }else if(Turn.ActualValue<-50)
//      {
//          Turn.ActualValue=-50;
//      }
      Turn.SetValue=0;                //车身保持在中间
      PID_POS(&Turn);
//      Balance_Aid();

    //初步设想：速度环并上（角度环和角加速度环的串级），以此实现差速效果
    // 角度环PD,位置式(引入卡尔曼滤波)

//     if(tr==5)//10ms计算一次角度环
//     {

      /*==================================角度环==================================*/
            //陀螺仪获取数据的两个函数会卡程序

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



         /*==================================角速度环==================================*/


        PID_Calc(&Gx,Roll.out, gyro_lpf );//期望值为0




//            //调死区用
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


//PID输出到电机
//num:电机的编号
//speed:PID总输出，可正可负,MAX=10000
//死区补偿：
//       满电时  Motor1---467   Motor2---410

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
//开环速度规划
void Speed_Ctr(uint16_t *speed,const uint8_t motorstate)
{
    switch(motorstate)
    {
        case 0://直行 出弯 加速
            *speed += 50;
            *speed = *speed>ZSMAX? (ZSMAX):(*speed);
            break;
        case 1://入弯减速
            *speed -= 100;
            *speed = *speed<SMIN? (SMIN):(*speed);
            break;
        default:
            break;
    }
}


#pragma section all restore
