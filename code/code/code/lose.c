#include "zf_common_headfile.h"

#include "repair.h"

#include "dajinfa.h"
#include "lose.h"
#include "saoxian.h"
#include <math.h>





//左上角为原点 向下为x轴正方向  向右为y轴正方向
//车的映射点
uint8 mapping_x=MT9V03X_H-10;
uint8 mapping_y=MT9V03X_W/2;

float distance;


KalmanFilter kf_distance={
        0.02, //估算协方差. 初始化值为 0.02
        0, //卡尔曼增益. 初始化值为 0
        0.1, //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏. 初始化值为 0.001
        1, //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好. 初始化值为 1
        0 //卡尔曼滤波器输出. 初始化值为 0
    };






float KalmanFilter_run(KalmanFilter *kfp, float input)
{
    //估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    kfp->P = kfp->P + kfp->Q;

    //卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    kfp->G = kfp->P / (kfp->P + kfp->R);

    //更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    kfp->Output = kfp->Output + kfp->G * (input - kfp->Output); //当前 估算值 = 上次 最优值

    //更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差。
    kfp->P = (1 - kfp->G) * kfp->P;

     return kfp->Output;
}




uint8 Lost_line_l;//左边丢线
uint8 Lost_line_r;


void  lose_midline()
{

    if (Search_Stop_Line>90)//截止行太低
    {
        return;
    }
    Lost_line_l=0;//丢线数清零
    Lost_line_r=0;

    uint16 i;//记录丢线树
    for(i=0;i<l_data_statics;i++)
    {
       if (points_l[i][0]<=border_min)
       {
           Lost_line_l+=1;
       }
    }
    for(i=0;i<r_data_statics;i++)
    {
        if (points_r[i][0]>=border_max)
        {
            Lost_line_r+=1;
        }
    }


//   printf("丢线数%d %d\r\n",Lost_line_l,Lost_line_r);

   //判断元素
   cross_fill();

    //求中线
   if (deviation==0)
   {
        for (i = Search_Stop_Line; i < Sweeping_line_start; i++)
        {
            Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;
            bin_Multi_threshold[i][Mid_Line[i]]=black;
        }
   }
   if (deviation==1)//左线偏移
   {
       for (i = Search_Stop_Line; i < Sweeping_line_start; i++)
       {
           Mid_Line[i] = Left_Line[i] + Standard_Road_Wide[i];
           if (Mid_Line[i]>=border_max)
           {
               Mid_Line[i]=border_max;
           }
           bin_Multi_threshold[i][Mid_Line[i]]=black;
       }

   }
   if (deviation==2)//右线偏移
   {
       for (i = Search_Stop_Line; i < Sweeping_line_start; i++)
       {
              Mid_Line[i] = Right_Line[i] - Standard_Road_Wide[i];
              if (Mid_Line[i]<=border_min)
              {
                  Mid_Line[i]=border_min;
              }
              bin_Multi_threshold[i][Mid_Line[i]]=black;
        }

    }

    //初始化追逐点
    uint8 chase_x=Search_Stop_Line+30;//追逐点是截止行下面一点
    uint8 chase_y=MT9V03X_W/2;//默认直线


     chase_y=Mid_Line[chase_x];
     distance=chase_y-mapping_y;//追逐点的横向距离
     KalmanFilter_run(&kf_distance,distance);

     //给输出加个激活函数，在0附近变化率低
//     float output=kf_distance.Output;
//     float amplitude= 90.0 * (PI / 180.0)*30;//1.57 放大30
//     if (output>amplitude)
//     {
//         output=amplitude;
//     }
//     else if (output<(-amplitude))
//     {
//         output=-amplitude;
//     }
//     output=tan(output/30)*30;//可以加个限幅
//     kf_distance.Output=output;
}

















