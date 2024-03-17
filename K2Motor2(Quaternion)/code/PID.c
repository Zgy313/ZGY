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

float dynzero=-1.0;//机械零点（动态零点，摩托车在转弯时需要有一定的倾斜角度）  -0.65
//u8 PIDCalFlag=0;
u8 tm=0,tr=0,tg=0,ty=0,ts=0;
extern float p;
extern float i;
extern float d;
float gyro_lpf=0;//角速度值
float gyro_z_out=0;
#pragma section all "cpu1_psram"
//将所有PID参数都初始化
void AllPID_Init(void)
{
    //各PID参数在这里修改
    //速度环PD（暂时不加，问题：加上后小车静止时无法保持平衡）
    //大轮速度环PI
//        POS_PID_Init(&Motor1,667.9,132.5,222.9,100,10000);
//        POS_PID_Init(&Motor2,973.5,193.5,291.5,100,10000);
    //小轮速度环PD
    //0.26 0.0008 0.228
    POS_PID_Init(&Motor1,0.26,0.2,0.31,0.6,90);
//    POS_PID_Init(&Motor1,0.26,0.0008,0.228,1,90);
//    POS_PID_Init(&Motor1,0.14,0.003,0.47,15,90);
//    POS_PID_Init(&Motor1,0.44,0.04,0.4,10,90);
    //转向环PID(Yaw角速度环，增量式) 参考参数 0.1 0.0018 0.03 100 9000
    //    POS_PID_Init(&Yaw,0.1,0.0048,0.03,100,9000);
    //角度环PID参考参数（位置式） p 9.6 i 0.0001 d 1.5
    //    POS_PID_Init(&Roll,20.6,0,1.58,1000,9000);
    // 25.6 0.34 2.92
    POS_PID_Init(&Roll,26.26,0.26,2.962,20,9000);
    //角速度环参考参数（增量式） p 0.81 i 0.152 d 7.8
    PID_Init(&Gx,1.2,0.035,19.3,8000);


    Motor1.SetValue=0;       //cm/s    30-40左右
//    Motor2.SetValue=0;
//    Yaw.SetValue=0;

    //卡尔曼滤波 初始化
    MotorFilter1.Kg=0.02f;
    MotorFilter1.LastP=0;
    MotorFilter1.Now_P=0;
    //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    MotorFilter1.Q=0.001f;
    MotorFilter1.R=0.543f;
    MotorFilter1.out=0;

    //卡尔曼滤波 初始化
    MotorFilter2.Kg=0.02f;
    MotorFilter2.LastP=0;
    MotorFilter2.Now_P=0;
    //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    MotorFilter2.Q=0.001f;
    MotorFilter2.R=0.543f;
    MotorFilter2.out=0;

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
    cfloat32 in;
    cfloat32 out;
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
    /*====================以下是采用FFT计算PID========================*/
    in.real=((pid->error-pid->lastError)*pid->kp)+((pid->error-2*pid->lastError+pid->prev_error)*pid->kd)+pid->error*pid->ki;
    in.imag = 0.0;
    Ifx_FftF32_radix2(&out, &in, 1);//第一个参数为输出变量地址，第二个为输入变量地址，第三个为计算次数（非数组一般都是1）（个人理解）
    pid->output+=out.real;
    /*=============================================================*/
    pid->prev_error=pid->lastError;//将上上次的error作为预测值
    pid->lastError=pid->error;//将旧error存起来

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output=pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output=-pid->maxOutput;
}

//位置式pid
void PID_POS(PID *PID)
{
//    cfloat32 in;
//    cfloat32 out;

    //当前偏差
    PID->ek=PID->SetValue-PID->ActualValue;
    //比例输出控制量
    PID->kpout=PID->Kp_pos*PID->ek;
    //积分输出控制量
    PID->kiout+=PID->ek;
    PID->kiout*=PID->ki_pos;
    //积分限幅
    if(PID->kiout > PID->maxIntegral)
//        PID->kiout=PID->maxIntegral;
        PID->kiout=0;
    else if(PID->kiout < -PID->maxIntegral)
//        PID->kiout=-PID->maxIntegral;
        PID->kiout=0;
    //微分输出控制量
    PID->kdout=PID->kd_pos*(PID->ek-PID->ek1);
    //PID控制器的输出控制量
    /*===================软件计算======================*/
//    in.real=(PID->Kp_pos*PID->ek)+( PID->kd_pos*(PID->ek-PID->ek1) )+PID->kiout;
//    in.imag=0;
//    Ifx_FftF32_radix2(&out, &in, 1);//第一个参数为输出变量地址，第二个为输入变量地址，第三个为计算次数（非数组一般都是1）（个人理解）
//    PID->out=out.real;
    /*===============================================*/
    PID->out=PID->kpout+PID->kiout+PID->kdout;
    //输出限幅
    if(PID->out > PID->maxOutput) PID->out=PID->maxOutput;
    else if(PID->out < -PID->maxOutput) PID->out=PID->maxOutput;
    //历史偏差量赋值
    PID->ek1=PID->ek;
}



//PID总输出函数(在PIT中断函数中调用)
void PID_Realize(void)
{
//   static uint8_t dir=0;

    gyro_lpf=0;
    imu660ra_get_acc();
    imu660ra_get_gyro();

    Motor1.ActualValue=(float)encoder_get_count(TIM2_ENCODER);
    Motor1.ActualValue=kalmanFilter(&MotorFilter1,Motor1.ActualValue);
    /*=================这里闭正常的速度环取负号======================*/
//    Motor2.ActualValue=-(float)encoder_get_count(TIM4_ENCODER);
    Motor2.ActualValue=(float)encoder_get_count(TIM4_ENCODER);
    Motor2.ActualValue=kalmanFilter(&MotorFilter2,Motor2.ActualValue);

    //速度环PD(增量式)
  if(tm==100)//100ms计算一次速度环
  {


      /*================================================================*/
//         Motor1.feedback=speed1.speed;
//         Motor2.feedback=-speed2.speed;//  电机1和电机2是对称的，编码器值相反
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
      //编码器测试代码
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

/*============================速度环===================================*/
      PID_POS(&Motor1);
//      PID_POS(&Motor2);

/*====================================================================*/
//        printf("%f,%f\n",Motor2.reference,Motor2.feedback);
      tm=0;
  }
    //程序卡住是因为定时器初始化放在了最前面，全部初始化还未完成便进入了中断，导致程序卡在了中断里,没初始化调用也会卡
    //转向加速度环
//    if(ty==10)
//    {
//          gyro_z_out=0;
//          gyro_z_out = lpf_operator(&lpf_current_gyro,imu660ra_gyro_transition(imu660ra_gyro_z),0.02)-0.027f;
////        Yaw.ActualValue=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_z));//减去零漂
//          Yaw.ActualValue=gyro_z_out;
////        if((Yaw.ActualValue<0.01)&&(Yaw.ActualValue>-0.01)) Yaw.ActualValue=0;
////        printf("%f\n",Yaw.ActualValue);
//        PID_POS(&Yaw);//期望值为0
////        printf("yaw=%f\n",Yaw.out);
////        seekfree_assistant_oscilloscope_data.channel_num=2;
////        seekfree_assistant_oscilloscope_data.data[0]=Yaw.SetValue;
////        seekfree_assistant_oscilloscope_data.data[1]=Yaw.ActualValue;
////        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
//        ty=0;
//    }


    //初步设想：速度环并上（角度环和角加速度环的串级），以此实现差速效果
    // 角度环PD,位置式(引入卡尔曼滤波)
     if(tr==5)//5ms计算一次角度环
     {
            //陀螺仪获取数据的两个函数会卡程序
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
     //角加速度环PI，增量式（低通滤波）
      if(tg==1)//1ms计算一次角加速度环
      {
            //          gyro_lpf=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_x));//减去零漂
            gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.001)+0.5;
            //          gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.005);
            //          Roll.out=0;//期望值0
            PID_Calc(&Gx,Roll.out, gyro_lpf);//期望值为0
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
        //输出(会卡程序)
//        PID_Out(1,2000);
//        PID_Out(2,2000);

//    printf("motor1out=%f\n",Motor1.output);
            PID_Out(1,-0-Gx.output);
            PID_Out(2,-0+Gx.output);

        //舵机辅助平衡
        if(ts==0)
        {
//            imu660ra_get_acc();
//            imu660ra_get_gyro();
//            IMU_Update(imu660ra_gyro_transition(imu660ra_gyro_x),imu660ra_gyro_transition(imu660ra_gyro_y),imu660ra_gyro_transition(imu660ra_gyro_z),imu660ra_acc_transition(imu660ra_acc_x),imu660ra_acc_transition(imu660ra_acc_y),imu660ra_acc_transition(imu660ra_acc_z));
//            ServoAid=FilterRoll+0.81f;
            Balance_Aid();

        }

}


//PID输出到电机
//num:电机的编号
//speed:PID总输出，可正可负,MAX=10000
//死区补偿：快没电时 Motor1---700   Motor2---1000
//       满电时  Motor1---650   Motor2---800
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
