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
u16 dynzero=8;//机械零点（动态零点，摩托车在转弯时需要有一定的倾斜角度）  8°
//u8 PIDCalFlag=0;
u8 tm=0,tr=0,tg=0,ty=0;
float gyro_lpf=0;//角速度值
#pragma section all "cpu1_psram"
//将所有PID参数都初始化
void AllPID_Init(void)
{
     //各PID参数在这里修改
    //速度环PI（暂时不加，问题：加上后小车静止时无法保持平衡）
//    PID_Init(&Motor1,1.48,0.65,17.6,10000);
//    PID_Init(&Motor2,12.4,12.2,31.2,10000);

    //转向环PID(Yaw角速度环，增量式) 参考参数 p 0.34 i0.001 d 0.083
    POS_PID_Init(&Yaw,0.0094,0.0001,0.008,1000,10000);
    //角度环PID参考参数（位置式） p 0.362 i 0.0001 d 0.028
    POS_PID_Init(&Roll,0.362,0.0001,0.028,1000,10000);
    //角速度环参考参数（增量式） p 0.81 i 0.152 d 7.8
    PID_Init(&Gx,0.81,0.152,7.89,9000);
//    Motor1.reference=-0;       //cm/s
//    Motor2.reference=-0;
    Yaw.SetValue=0;

    //卡尔曼滤波 初始化
    MotorFilter1.Kg=0.02f;
    MotorFilter1.LastP=0;
    MotorFilter1.Now_P=0;
    //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    MotorFilter1.Q=0.002f;
    MotorFilter1.R=2.2f;
    MotorFilter1.out=0;

    //卡尔曼滤波 初始化
    MotorFilter2.Kg=0.02f;
    MotorFilter2.LastP=0;
    MotorFilter2.Now_P=0;
    //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    MotorFilter2.Q=0.01f;
    MotorFilter2.R=3.2f;
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

    //当前偏差
    PID->ek=PID->SetValue-PID->ActualValue;
    //比例输出控制量
    PID->kpout=PID->Kp_pos*PID->ek;
    //积分输出控制量
    PID->kiout+=PID->ek;
    PID->kiout*=PID->ki_pos;
    //积分限幅
    if(PID->kiout > PID->maxIntegral)
        PID->kiout=0;
    else if(PID->kiout < -PID->maxIntegral) PID->kiout=0;
    //微分输出控制量
    PID->kdout=PID->kd_pos*(PID->ek-PID->ek1);
    //PID控制器的输出控制量
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
    //程序卡住是因为定时器初始化放在了最前面，全部初始化还未完成便进入了中断，导致程序卡在了中断里,没初始化调用也会卡
    //转向加速度环
    if(ty==3)
    {
//        gyro_z_out = lpf_operator(&lpf_current_gyro,imu660ra_gyro_transition(imu660ra_gyro_z),0.005)-1.533;
        Yaw.ActualValue+=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_z))-0.11;//减去零漂
//        printf("%f\n",Yaw.ActualValue);
        if((Yaw.ActualValue<0.01)&&(Yaw.ActualValue>-0.01)) Yaw.ActualValue=0;
        PID_POS(&Yaw);//期望值为0
//        printf("yaw=%f\n",Yaw.out);
        ty=0;
    }


    //初步设想：速度环并上（角度环和角加速度环的串级），以此实现差速效果
    // 角度环PD,位置式(引入卡尔曼滤波)
     if(tr==2)//10ms计算一次角度环
     {
         //陀螺仪获取数据的两个函数会卡程序

         Roll.ActualValue+=Kalman_getAngle(0.01,imu660ra_acc_transition(imu660ra_acc_y),imu660ra_gyro_transition(imu660ra_gyro_x));
//         printf("%f\n",Roll.ActualValue);
         Roll.SetValue=dynzero+Yaw.out;
//         Roll.SetValue=dynzero+0;
         PID_POS(&Roll);
//         printf("roll=%f\n",Roll.out);
         tr=0;
     }
     //角加速度环PI，增量式（低通滤波）
      if(tg==1)//5ms计算一次角加速度环
      {
//          gyro_lpf=kalmanFilter(&gyro_z_str,imu660ra_gyro_transition(imu660ra_gyro_z));//减去零漂
          gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.005)-1.29;
//          gyro_lpf = lpf_operator(&lpf_current_gyro,imu660ra_gyro_x,0.005);
          PID_Calc(&Gx,Roll.out, gyro_lpf);//期望值为0
//          printf("%f\n",gyro_lpf);
//          printf("Gx=%f\n",Gx.output);
          tg=0;

      }

      //速度环PI(增量式)
//    if(tm==6)//30ms计算一次速度环
//    {


        /*================================================================*/
//         Motor1.feedback=speed1.speed;
//         Motor2.feedback=-speed2.speed;//  电机1和电机2是对称的，编码器值相反
         /*================================================================*/
/*==============================================================================================================*/
        //编码器测试代码
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

/*============================速度环===================================*/
//        PID_Calc(&Motor1, Motor1.reference, Motor1.feedback);
//        PID_Calc(&Motor2, Motor2.reference, Motor2.feedback);
//        Motor1.output=kalmanFilter(&MotorFilter1,Motor1.output);
//        Motor2.output=kalmanFilter(&MotorFilter2,Motor2.output);
/*====================================================================*/
//        printf("%f,%f\n",Motor2.reference,Motor2.feedback);
//        tm=0;
//    }


        //输出(会卡程序)
//        PID_Out(1,Motor1.reference*20+Motor1.output-Gx.output);
//        PID_Out(2,Motor2.reference*20+Motor2.output+Gx.output);

//    printf("motor1out=%f\n",Motor1.output);
            PID_Out(1,0-Gx.output);
            PID_Out(2,0+Gx.output);

}


//PID输出到电机
//num:电机的编号
//speed:PID总输出，可正可负,MAX=10000
//死区补偿：快没电时 Motor1---700   Motor2---1000
//       满电时  Motor1---650   Motor2---800
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
