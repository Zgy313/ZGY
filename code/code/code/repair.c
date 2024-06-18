#include "zf_common_headfile.h"


#include "dajinfa.h"
#include "time.h"
#include "saoxian.h"
#include "lose.h"




/*startY要大于endY
 * K_Add_Line
 * Find_Left_Down_Point
 * Find_Right_Down_Point
 * Find_Left_Down_Point
 * Find_Right_up_Point
 * */


//通过斜率，定点补线 修改的Left_Line   startY要小于endY,一般只有end会出界
void K_Add_Line(float k,uint8 startX,uint8 startY,uint8 endY,uint8 Line[MT9V03X_H])
{
    uint8 i;
    if (endY<=2)
    {
        endY=2;
    }
    for(i=startY;i>=endY;i--)
    {
        Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
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




//输入起始点，终点坐标, 修改左边线 默认startY>endY
void Draw_Line_Left(uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    uint8 i,x;

        for (i = startY; i <= endY; i--)//纵向补线，保证每一行都有黑点
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//两点式变形
            if(x>=MT9V03X_W-2)
                x=MT9V03X_W-2;
            else if (x<=1)
                x=1;
            Left_Line[i]=x;
        }
}







//计算斜率
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



//计算斜率和截距
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
    //计算各个平均数
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);
    }
    /*计算斜率*/
    *slope_rate = Slope_Calculate(start, end, border);//斜率
    *intercept = y_average - (*slope_rate)*x_average;//截距
}




//边界撕裂，感觉能上生长方向
uint8 Find_Left_Down_Point(uint8 start,uint8 end)//找左下角点，返回值是角点所在的行数
{
    uint8 i;
    for(i=start;i>=end;i--)
    {
        if(//只找第一个符合条件的点
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
           Left_Line[i]-Left_Line[i-2]>=5&&
           Left_Line[i]-Left_Line[i-3]>=5&&
           Left_Line[i]-Left_Line[i-4]>=5)
        {
            return i;//获取行数即可
        }
    }
    return 0;
}



//寻找右下点
uint8 Find_Right_Down_Point(uint8 start,uint8 end)
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])>=-5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])>=-5&&
           abs(Right_Line[i+2]-Right_Line[i+3])>=-5&&//下面的点在5以内，上面的点在5以外
           Right_Line[i]-Right_Line[i-2]<=-5&&
           Right_Line[i]-Right_Line[i-3]<=-5&&
           Right_Line[i]-Right_Line[i-4]<=-5)
        {
            return i;//获取行数即可
        }
    }
    return 0;
}



uint8 Find_Right_up_Point(uint8 start,uint8 end)//找右上角点，返回值是角点所在的行数
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(//只找第一个符合条件的点
                Right_Line[i]-Right_Line[i+2]<=-5&&//角点的阈值可以更改
                Right_Line[i]-Right_Line[i+3]<=-5&&
                Right_Line[i]-Right_Line[i+4]<=-5&&
                abs(Right_Line[i-1]-Right_Line[i-2])>=-5&&
                abs(Right_Line[i-2]-Right_Line[i-3])>=-5&&
                abs(Right_Line[i-3]-Right_Line[i-4])>=-5)
        {
            return i;//获取行数即可
            ;
        }
    }
    return 0;
}



uint8 Find_Lift_up_Point(uint8 start,uint8 end)//找左上角点，返回值是角点所在的行数
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(//只找第一个符合条件的点
                Right_Line[i]-Left_Line[i+2]>=5&&//角点的阈值可以更改
                Left_Line[i]-Left_Line[i+3]>=5&&
                Left_Line[i]-Left_Line[i+4]>=5&&
                abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
                abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
                abs(Left_Line[i-3]-Left_Line[i-4])<=5)
        {
            return i;//获取行数即可
            ;
        }
    }
    return 0;
}




//单调性突变检测 左边
uint8 Monotonicity_Change_Left(uint8 start,uint8 end)//单调性改变，返回值是单调性改变点所在的行数
{
    uint8 i;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {

        if(Left_Line[i] >Left_Line[i+5]&&Left_Line[i] >Left_Line[i-5]&&
        Left_Line[i] >Left_Line[i+4]&&Left_Line[i] >Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            return i;

        }
    }
    return 0;
}



//左赛道连续性检测
uint8 Continuity_Change_Left(uint8 start,uint8 end)
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(abs(Left_Line[i]-Left_Line[i-1])>=5)//连续性阈值是5，可更改
       {
            return i;
       }
    }
    return 0;
}

//右赛道连续性检测
uint8 Continuity_Change_Right(uint8 start,uint8 end)
{
    uint8 i;

    for(i=start;i>=end;i--)
    {
        if(abs(Right_Line[i]-Right_Line[i-1])>=5)//连续性阈值是5，可更改
       {
            return i;
       }
    }
    return 0;
}

uint8 sign;
uint8 sign_cross;//十字计数
uint8 sign_ring;//圆环计数

uint8 cross_1;//十字遇
uint8 cross_2;//十字出

uint8 ring_1;//圆环 进 左下加弧
uint8 ring_2;//出圆环 左上点

uint8 deviation;//用偏移当中线
//0原始  1左线偏移  右线偏移

//元素判断
void cross_fill()
{
    if (!ring_1)
    {
        deviation=0;//初始化偏移标志位
        //遇圆环了就直接次序左线偏移
    }
    if (!ring_2)
    {
        deviation=0;
    }
    uint8 cross_down_l = 0;//左下拐点
    uint8 cross_down_r = 0;//右下拐点
    uint8 cross_up_l=0;//左上拐点
    uint8 cross_up_r=0;
    uint8 continuity_Left=0;//左赛道断点
    uint8 continuity_Right=0;//右赛道断点

    uint8 Monotonicity_Left=0;//左单调性点

    if (Lost_line_l>10 && Lost_line_r>10 && Search_Stop_Line<40 && Lost_line_l<100 && Lost_line_r<100)
//    if (Search_Stop_Line<40 && Lost_line_l<100 && Lost_line_r<100)
    {
        //俩边出现丢线，且截止行靠前
        printf("两边丢线严重\n");
        cross_down_l=Find_Left_Down_Point(95,10);//往下走20个点用来求斜率
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
                printf("左下%d\n",cross_down_l);
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
                printf("右下%d\n",cross_down_r);
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
                printf("左上%d\n",cross_up_l);
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
                printf("右上%d\n",cross_up_r);
            }
        }



        if (cross_down_l && cross_down_r && cross_up_l && cross_up_r)
        {
            printf("遇十\n");
            k=Slope_Calculate(cross_down_l+5,cross_down_l+20,Left_Line);
            K_Add_Line(k,Left_Line[cross_down_l] ,cross_down_l, cross_down_l-80,Left_Line);
            k=Slope_Calculate(cross_down_r+5,cross_down_l+20,Right_Line);
            K_Add_Line(k,Right_Line[cross_down_r] ,cross_down_r, cross_down_r-80,Right_Line);
            //补左线，补右线
            //由于追逐点靠前，只用找到下面的拐点就能顺利过去
            cross_1=1;

        }
        //转不过来这样写，如果转的过来感觉都不用处理

        if (cross_1 && cross_down_l && cross_up_l && !cross_up_r)
        {
            printf("出十\n");
            k=Slope_Calculate(cross_down_l+5,cross_down_l+20,Left_Line);
            K_Add_Line(k,Left_Line[cross_down_l] ,cross_down_l, cross_down_l-80,Left_Line);
            cross_2=1;
        }
    }


    //处理左圆环
    //圆环1 左断点 左单调点 右连续
    if (Lost_line_l>20 && Lost_line_r<5 && Search_Stop_Line<40)
    {
        printf("左边严重丢线\n");
        continuity_Left=Continuity_Change_Left(95,10);
        Monotonicity_Left=Monotonicity_Change_Left(95,10);
        continuity_Right=Continuity_Change_Right(95,10);
        if (continuity_Left)
        {
            printf("左断电%d\n",cross_down_l);
        }
        if (Monotonicity_Left)
        {
            printf("左单调点\n");
        }
        if (continuity_Left && Monotonicity_Left && !continuity_Right && (continuity_Left>Monotonicity_Left))
        {
            printf("圆环1\n");
            Draw_Line_Left(Left_Line[continuity_Left],continuity_Left,Left_Line[Monotonicity_Left],Monotonicity_Left);

            deviation=1;//左线偏移
            ring_1=1;
        }
    }

    //感觉不用找左上点，直接左线偏移进圆环
    //圆环2
    //找左拐点 然后右线偏移当中线

}




void clear_sign(uint16 *cnt)//清楚十字标志为
{
    if (*cnt==(10000/25))//10秒清楚十字标志位
    {
        if (cross_1)
        {
                sign_cross+=1;
                sign=sign_cross;
        }
        if (cross_1 && cross_2)
        {
            cross_1=0;//十字下
            cross_2=0;//十字出
        }
    }

    if (*cnt>=(20000/25))//二十秒清楚圆环标志位
    {
        if (ring_1)
        {
            sign_ring+=1;
            sign=sign_ring+100;
        }
        if (ring_1 && ring_2)
        {
            ring_1=0;//圆环 进
            ring_2=0;//圆环 出
        }
        *cnt=0;
    }
}







