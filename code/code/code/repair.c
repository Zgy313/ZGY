#include "zf_common_headfile.h"


#include "dajinfa.h"
#include "time.h"
#include "saoxian.h"
#include "lose.h"




/*startYҪ����endY
 * K_Add_Line
 * Find_Left_Down_Point
 * Find_Right_Down_Point
 * Find_Left_Down_Point
 * Find_Right_up_Point
 * */


//ͨ��б�ʣ����㲹�� �޸ĵ�Left_Line   startYҪС��endY,һ��ֻ��end�����
void K_Add_Line(float k,uint8 startX,uint8 startY,uint8 endY,uint8 Line[MT9V03X_H])
{
    uint8 i;
    if (endY<=2)
    {
        endY=2;
    }
    for(i=startY;i>=endY;i--)
    {
        Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)���Σ�x=(y-y1)/k+x1
            if(Line[i]>=MT9V03X_W-2)
            {
                Line[i]=MT9V03X_W-2;
            }
            else if(Line[i]<=1)
            {
                Line[i]=1;
            }
            bin_Multi_threshold[i][Line[i]]=black;
    }

}




//������ʼ�㣬�յ�����, �޸������ Ĭ��startY>endY
void Draw_Line_Left(uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    uint8 i,x;

        for (i = startY; i <= endY; i--)//�����ߣ���֤ÿһ�ж��кڵ�
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//����ʽ����
            if(x>=MT9V03X_W-2)
                x=MT9V03X_W-2;
            else if (x<=1)
                x=1;
            Left_Line[i]=x;
        }
}







//����б��
float Slope_Calculate(uint8 start, uint8 end, uint8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    for (i = start; i < end; i++)
    {
        ysum += i;
        xsum += border[i];
        xysum += border[i] * i;
        x2sum += border[i] * border[i];
    }
    return ((end - start)*xysum - xsum * ysum) / ((end - start)*x2sum - xsum * xsum+1e-3);
}



//����б�ʺͽؾ�
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;
    uint16 xsum = 0, ysum = 0;
    float y_average, x_average;
    for (i = start; i < end; i++)
    {
        ysum += i;
        xsum += border[i];
        num++;
    }
    //�������ƽ����
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);
    }
    /*����б��*/
    *slope_rate = Slope_Calculate(start, end, border);//б��
    *intercept = y_average - (*slope_rate)*x_average;//�ؾ�
}




//�߽�˺�ѣ��о�������������
uint8 Find_Left_Down_Point(uint8 start,uint8 end)//�����½ǵ㣬����ֵ�ǽǵ����ڵ�����
{
    uint8 i;
    for(i=start;i>=end;i--)
    {
        if(//ֻ�ҵ�һ�����������ĵ�
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//�ǵ����ֵ���Ը���
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
           Left_Line[i]-Left_Line[i-2]>=5&&
           Left_Line[i]-Left_Line[i-3]>=5&&
           Left_Line[i]-Left_Line[i-4]>=5)
        {
            return i;//��ȡ��������
        }
    }
    return 0;
}



//Ѱ�����µ�
uint8 Find_Right_Down_Point(uint8 start,uint8 end)
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(//ֻ�ҵ�һ�����������ĵ�
           abs(Right_Line[i]-Right_Line[i+1])>=-5&&//�ǵ����ֵ���Ը���
           abs(Right_Line[i+1]-Right_Line[i+2])>=-5&&
           abs(Right_Line[i+2]-Right_Line[i+3])>=-5&&//����ĵ���5���ڣ�����ĵ���5����
           Right_Line[i]-Right_Line[i-2]<=-5&&
           Right_Line[i]-Right_Line[i-3]<=-5&&
           Right_Line[i]-Right_Line[i-4]<=-5)
        {
            return i;//��ȡ��������
        }
    }
    return 0;
}



uint8 Find_Right_up_Point(uint8 start,uint8 end)//�����Ͻǵ㣬����ֵ�ǽǵ����ڵ�����
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(//ֻ�ҵ�һ�����������ĵ�
                Right_Line[i]-Right_Line[i+2]<=-5&&//�ǵ����ֵ���Ը���
                Right_Line[i]-Right_Line[i+3]<=-5&&
                Right_Line[i]-Right_Line[i+4]<=-5&&
                abs(Right_Line[i-1]-Right_Line[i-2])>=-5&&
                abs(Right_Line[i-2]-Right_Line[i-3])>=-5&&
                abs(Right_Line[i-3]-Right_Line[i-4])>=-5)
        {
            return i;//��ȡ��������
            ;
        }
    }
    return 0;
}



uint8 Find_Lift_up_Point(uint8 start,uint8 end)//�����Ͻǵ㣬����ֵ�ǽǵ����ڵ�����
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(//ֻ�ҵ�һ�����������ĵ�
                Right_Line[i]-Left_Line[i+2]>=5&&//�ǵ����ֵ���Ը���
                Left_Line[i]-Left_Line[i+3]>=5&&
                Left_Line[i]-Left_Line[i+4]>=5&&
                abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
                abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
                abs(Left_Line[i-3]-Left_Line[i-4])<=5)
        {
            return i;//��ȡ��������
            ;
        }
    }
    return 0;
}




//������ͻ���� ���
uint8 Monotonicity_Change_Left(uint8 start,uint8 end)//�����Ըı䣬����ֵ�ǵ����Ըı�����ڵ�����
{
    uint8 i;
    for(i=start;i>=end;i--)//���ȡǰ5��5���ݣ�����ǰ������뷶Χ��Ҫ��
    {

        if(Left_Line[i] >Left_Line[i+5]&&Left_Line[i] >Left_Line[i-5]&&
        Left_Line[i] >Left_Line[i+4]&&Left_Line[i] >Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {//�ͺܱ����������������ǰ5����5�����ģ��Ǿ��ǵ���ͻ���
            return i;

        }
    }
    return 0;
}



//�����������Լ��
uint8 Continuity_Change_Left(uint8 start,uint8 end)
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(abs(Left_Line[i]-Left_Line[i-1])>=5)//��������ֵ��5���ɸ���
       {
            return i;
       }
    }
    return 0;
}

//�����������Լ��
uint8 Continuity_Change_Right(uint8 start,uint8 end)
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5)//��������ֵ��5���ɸ���
       {
            return i;
       }
    }
    return 0;
}

uint8 sign;
uint8 sign_cross;//ʮ�ּ���
uint8 sign_ring;//Բ������

uint8 cross_1;//ʮ����
uint8 cross_2;//ʮ�ֳ�

uint8 ring_1;//Բ�� �� ���¼ӻ�
uint8 ring_2;//��Բ�� ���ϵ�

uint8 deviation;//��ƫ�Ƶ�����
//0ԭʼ  1����ƫ��  ����ƫ��

//Ԫ���ж�
void cross_fill()
{
    if (!ring_1)
    {
        deviation=0;//��ʼ��ƫ�Ʊ�־λ
        //��Բ���˾�ֱ�Ӵ�������ƫ��
    }
    if (!ring_2)
    {
        deviation=0;
    }
    uint8 cross_down_l = 0;//���¹յ�
    uint8 cross_down_r = 0;//���¹յ�
    uint8 cross_up_l=0;//���Ϲյ�
    uint8 cross_up_r=0;
    uint8 continuity_Left=0;//�������ϵ�
    uint8 continuity_Right=0;//�������ϵ�

    uint8 Monotonicity_Left=0;//�󵥵��Ե�

    if (Lost_line_l>10 && Lost_line_r>10 && Search_Stop_Line<40 && Lost_line_l<100 && Lost_line_r<100)
//    if (Search_Stop_Line<40 && Lost_line_l<100 && Lost_line_r<100)
    {
        //���߳��ֶ��ߣ��ҽ�ֹ�п�ǰ
        printf("���߶�������\n");
        cross_down_l=Find_Left_Down_Point(95,10);//������20����������б��
        cross_down_r=Find_Right_Down_Point(95,10);
        cross_up_l=Find_Lift_up_Point(95,10);
        cross_up_r=Find_Right_up_Point(95,10);

        float k;
        if (cross_down_l)
        {
            if (Left_Line[cross_down_l]>MT9V03X_W/2)
            {
                cross_down_l=0;
            }
            else
            {
                printf("����%d\n",cross_down_l);
            }
        }
        if (cross_down_r)
        {
            if (Right_Line[cross_down_r]<MT9V03X_W/2)
            {
                cross_down_r=0;
            }
            else
            {
                printf("����%d\n",cross_down_r);
            }
        }
        if (cross_up_l)
        {
            if (Left_Line[cross_up_l]>MT9V03X_W/2)
            {
                cross_up_l=0;
            }
            else
            {
                printf("����%d\n",cross_up_l);
            }
        }
        if (cross_up_r)
        {
            if (Right_Line[cross_up_r]<MT9V03X_W/2)
            {
                cross_up_r=0;
            }
            else
            {
                printf("����%d\n",cross_up_r);
            }
        }



        if (cross_down_l && cross_down_r && cross_up_l && cross_up_r)
        {
            printf("��ʮ\n");
            k=Slope_Calculate(cross_down_l+5,cross_down_l+20,Left_Line);
            K_Add_Line(k,Left_Line[cross_down_l] ,cross_down_l, cross_down_l-80,Left_Line);
            k=Slope_Calculate(cross_down_r+5,cross_down_l+20,Right_Line);
            K_Add_Line(k,Right_Line[cross_down_r] ,cross_down_r, cross_down_r-80,Right_Line);
            //�����ߣ�������
            //����׷��㿿ǰ��ֻ���ҵ�����Ĺյ����˳����ȥ
            cross_1=1;

        }
        //ת����������д�����ת�Ĺ����о������ô���

        if (cross_1 && cross_down_l && cross_up_l && !cross_up_r)
        {
            printf("��ʮ\n");
            k=Slope_Calculate(cross_down_l+5,cross_down_l+20,Left_Line);
            K_Add_Line(k,Left_Line[cross_down_l] ,cross_down_l, cross_down_l-80,Left_Line);
            cross_2=1;
        }
    }


    //������Բ��
    //Բ��1 ��ϵ� �󵥵��� ������
    if (Lost_line_l>20 && Lost_line_r<5 && Search_Stop_Line<40)
    {
        printf("������ض���\n");
        continuity_Left=Continuity_Change_Left(95,10);
        Monotonicity_Left=Monotonicity_Change_Left(95,10);
        continuity_Right=Continuity_Change_Right(95,10);
        if (continuity_Left)
        {
            printf("��ϵ�%d\n",cross_down_l);
        }
        if (Monotonicity_Left)
        {
            printf("�󵥵���\n");
        }
        if (continuity_Left && Monotonicity_Left && !continuity_Right && (continuity_Left>Monotonicity_Left))
        {
            printf("Բ��1\n");
            Draw_Line_Left(Left_Line[continuity_Left],continuity_Left,Left_Line[Monotonicity_Left],Monotonicity_Left);

            deviation=1;//����ƫ��
            ring_1=1;
        }
    }

    //�о����������ϵ㣬ֱ������ƫ�ƽ�Բ��
    //Բ��2
    //����յ� Ȼ������ƫ�Ƶ�����

}




void clear_sign(uint16 *cnt)//���ʮ�ֱ�־Ϊ
{
    if (*cnt==(10000/25))//10�����ʮ�ֱ�־λ
    {
        if (cross_1)
        {
                sign_cross+=1;
                sign=sign_cross;
        }
        if (cross_1 && cross_2)
        {
            cross_1=0;//ʮ����
            cross_2=0;//ʮ�ֳ�
        }
    }

    if (*cnt>=(20000/25))//��ʮ�����Բ����־λ
    {
        if (ring_1)
        {
            sign_ring+=1;
            sign=sign_ring+100;
        }
        if (ring_1 && ring_2)
        {
            ring_1=0;//Բ�� ��
            ring_2=0;//Բ�� ��
        }
        *cnt=0;
    }
}







