#include "zf_common_headfile.h"

#include "repair.h"

#include "dajinfa.h"
#include "lose.h"
#include "saoxian.h"
#include <math.h>





//���Ͻ�Ϊԭ�� ����Ϊx��������  ����Ϊy��������
//����ӳ���
uint8 mapping_x=MT9V03X_H-10;
uint8 mapping_y=MT9V03X_W/2;

float distance;


KalmanFilter kf_distance={
        0.02, //����Э����. ��ʼ��ֵΪ 0.02
        0, //����������. ��ʼ��ֵΪ 0
        0.1, //��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵. ��ʼ��ֵΪ 0.001
        1, //��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��. ��ʼ��ֵΪ 1
        0 //�������˲������. ��ʼ��ֵΪ 0
    };






float KalmanFilter_run(KalmanFilter *kfp, float input)
{
    //����Э����̣���ǰ ����Э���� = �ϴθ��� Э���� + ��������Э����
    kfp->P = kfp->P + kfp->Q;

    //���������淽�̣���ǰ ���������� = ��ǰ ����Э���� / ����ǰ ����Э���� + ��������Э���
    kfp->G = kfp->P / (kfp->P + kfp->R);

    //��������ֵ���̣���ǰ ����ֵ = ��ǰ ����ֵ + ���������� * ����ǰ ����ֵ - ��ǰ ����ֵ��
    kfp->Output = kfp->Output + kfp->G * (input - kfp->Output); //��ǰ ����ֵ = �ϴ� ����ֵ

    //���� Э���� = ��1 - ���������棩 * ��ǰ ����Э���
    kfp->P = (1 - kfp->G) * kfp->P;

     return kfp->Output;
}




uint8 Lost_line_l;//��߶���
uint8 Lost_line_r;


void  lose_midline()
{

    if (Search_Stop_Line>90)//��ֹ��̫��
    {
        return;
    }
    Lost_line_l=0;//����������
    Lost_line_r=0;

    uint16 i;//��¼������
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


//   printf("������%d %d\r\n",Lost_line_l,Lost_line_r);

   //�ж�Ԫ��
   cross_fill();

    //������
   if (deviation==0)
   {
        for (i = Search_Stop_Line; i < Sweeping_line_start; i++)
        {
            Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;
            bin_Multi_threshold[i][Mid_Line[i]]=black;
        }
   }
   if (deviation==1)//����ƫ��
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
   if (deviation==2)//����ƫ��
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

    //��ʼ��׷���
    uint8 chase_x=Search_Stop_Line+30;//׷����ǽ�ֹ������һ��
    uint8 chase_y=MT9V03X_W/2;//Ĭ��ֱ��


     chase_y=Mid_Line[chase_x];
     distance=chase_y-mapping_y;//׷���ĺ������
     KalmanFilter_run(&kf_distance,distance);

     //������Ӹ����������0�����仯�ʵ�
//     float output=kf_distance.Output;
//     float amplitude= 90.0 * (PI / 180.0)*30;//1.57 �Ŵ�30
//     if (output>amplitude)
//     {
//         output=amplitude;
//     }
//     else if (output<(-amplitude))
//     {
//         output=-amplitude;
//     }
//     output=tan(output/30)*30;//���ԼӸ��޷�
//     kf_distance.Output=output;
}

















