#include "zf_common_headfile.h"
#include "dajinfa.h"
#include "saoxian.h"


uint8 Left_Line[MT9V03X_H]; //左边线数组
uint8 Right_Line[MT9V03X_H];//右边线数组

uint8 Mid_Line[MT9V03X_H];  //中线数组
uint8 White_Column[MT9V03X_W];//每列白列长度
uint8 Search_Stop_Line;     //搜索截止行

uint16 l_data_statics;//统计左边
uint16 r_data_statics;//统计右边


uint8 points_l[max_length][2] ;//左线 x,y
uint8 points_r[max_length][2];//右线 x,y
uint16 dir_r[max_length];//用来存储右边生长方向
uint16 dir_l[max_length];//用来存储左边生长方向
uint16 l_data_statics;//统计左边找到点的个数
uint16 r_data_statics;//统计右边找到点的个数

uint8 Boundry_Start_Left;   //左右边界起始点
uint8 Boundry_Start_Right;

uint8 Sweeping_line_start=MT9V03X_H-2;//扫线开始行
uint8 Standard_Road_Wide[MT9V03X_H];//标准赛宽数组


void init()
{
    uint8 i;
    for (i=0;i<MT9V03X_H;i++)
    {
        Left_Line[i]=border_min;
        Right_Line[i]=border_max;
        Mid_Line[i]=0;
        //左右边线默认和黑框重合
    }
    for (i=0;i<MT9V03X_W;i++)
    {
        White_Column[i]=0;//初始化最长白列
    }

    Search_Stop_Line=0;     //搜索截止行
    l_data_statics=0;//统计左边
    r_data_statics=0;//统计右边


    uint16 j;
    for (j=0;j<max_length;j++)
    {
        dir_r[j]=0;
        dir_l[j]=0;
        points_l[j][0]=0;
        points_l[j][1]=0;
        points_r[j][0]=0;
        points_r[j][0]=0;

    }
    l_data_statics=0;//统计左边找到点的个数
    r_data_statics=0;//统计右边找到点的个数

    Boundry_Start_Left=0;   //左右边界起始点
    Boundry_Start_Right=0;  //第一个非丢线点,常规边界起始点
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5//此参数可根据自己的需求调节
#define threshold_min   255*2//此参数可根据自己的需求调节
void image_filter()//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < MT9V03X_H - 1; i++)
    {
        for (j = 1; j < (MT9V03X_W - 1); j++)
        {
            //统计八个方向的像素值
            num =
                    bin_Multi_threshold[i - 1][j - 1] + bin_Multi_threshold[i - 1][j] + bin_Multi_threshold[i - 1][j + 1]
                + bin_Multi_threshold[i][j - 1] + bin_Multi_threshold[i][j + 1]
                + bin_Multi_threshold[i + 1][j - 1] + bin_Multi_threshold[i + 1][j] + bin_Multi_threshold[i + 1][j + 1];


            if (num >= threshold_max && bin_Multi_threshold[i][j] == 0)
            {

                bin_Multi_threshold[i][j] = 255;//

            }
            if (num <= threshold_min && bin_Multi_threshold[i][j] == 255)
            {

                bin_Multi_threshold[i][j] = 0;//

            }

        }
    }

}



//给图像画一个黑框
void image_draw_rectan()
{
    uint8 i = 0;
    for (i = 0; i < MT9V03X_H; i++)
    {
        bin_Multi_threshold[i][0] = 0;
        bin_Multi_threshold[i][1] = 0;

        bin_Multi_threshold[i][MT9V03X_W - 1] = 0;
        bin_Multi_threshold[i][MT9V03X_W - 2] = 0;

    }
    for (i = 0; i < MT9V03X_W; i++)
    {
        bin_Multi_threshold[0][i] = 0;
        bin_Multi_threshold[1][i] = 0;

    }
}


//得到最长白列
void get_zuibailie()
{
        int16 i;
        uint8 j;
        uint8 start_column=20;//最长白列的搜索区间
        uint8 end_column=MT9V03X_W-20;
        //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
        for (j =start_column; j<=end_column; j++)
        {
             for (i = Sweeping_line_start; i >= 2; i--)//从扫线起始行扫到填的黑框
             {
                    if(bin_Multi_threshold[i][j] == 0)
                        break;
                    else
                        White_Column[j]++;
              }
        }

        //从左到右找左边最长白列
        Search_Stop_Line =0;
        for(j=start_column;j<=end_column;j++)
        {
             if (Search_Stop_Line < White_Column[j])//找最长的那一列
             {
                 Search_Stop_Line = White_Column[j];//0是白列长度

             }
        }
        Search_Stop_Line=MT9V03X_H-Search_Stop_Line;
}


//寻找两个边界的边界点作为八邻域循环的起始点
uint8 start_point_l[2] ;//左边起点的x，y值
uint8 start_point_l_r[2];//跑道移动到中线的一侧
uint8 start_point_r[2];//右边起点的x，y值
uint8 start_point_r_l[2];
uint8 get_start_point(uint8 start_row)
{
    uint8 i;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_l_r[0]=0;
    start_point_l_r[1]=0;

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

    start_point_r_l[0]=0;
    start_point_r_l[1]=0;

    //从中间往左边，先找起点
    for (i = MT9V03X_W / 2; i > 0; i--)
    {

        if (bin_Multi_threshold[start_row][i] == 0 && bin_Multi_threshold[start_row][i - 1] == 255)
        {
            start_point_l_r[0]=i;
            start_point_l_r[1]=start_row;
        }
        if (bin_Multi_threshold[start_row][i] == 255 && bin_Multi_threshold[start_row][i - 1] == 0)//再往左走就是黑
        {
            start_point_l[0] = i;//x
            start_point_l[1] = start_row;//y
            break;
        }
    }

    for (i = MT9V03X_W / 2; i <MT9V03X_W-1; i++)
    {

        if (bin_Multi_threshold[start_row][i] == 0 && bin_Multi_threshold[start_row][i + 1] == 255)
        {
            start_point_r_l[0]=i;
            start_point_r_l[1]=start_row;
        }
        if (bin_Multi_threshold[start_row][i] == 255 && bin_Multi_threshold[start_row][i + 1] == 0)
        {
            start_point_r[0] = i;//x
            start_point_r[1] = start_row;//y
            break;
        }
    }

    if(start_point_r[0]&&start_point_l[0])
    {
        return 1;
    }
    else if(start_point_l[0] && start_point_l_r[0] && !start_point_r[0] && !start_point_r_l[0] )
    {
        start_point_r[0]=start_point_l_r[0];
        start_point_r[1]=start_point_l_r[1];
        return 1;
    }
    else if(!start_point_l[0] && !start_point_l_r[0] && start_point_r[0] && start_point_r_l[0] )
    {
        start_point_l[0]=start_point_r_l[0];
        start_point_l[1]=start_point_r_l[1];
        return 1;
    }

    return 0;

}




//八邻域
void search_l_r()
{

    uint16 i = 0, j = 0;
    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };//用来存储中心点周围的八个点的坐标
    uint8 index_l = 0;//索引下标
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };//中心坐标点

    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}};
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };

    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}};
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    center_point_l[0] = start_point_l[0];//x
    center_point_l[1] = start_point_l[1];//y
    center_point_r[0] = start_point_r[0];//x
    center_point_r[1] = start_point_r[1];//y

    uint16 break_flag=max_length;
    //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }//记录周围八个点的坐标

        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        r_data_statics++;//索引加一


        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            //image[y][x]细节
            if (bin_Multi_threshold[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && bin_Multi_threshold[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[i][0];
                temp_l[index_l][1] = search_filds_l[i][1];
                index_l++;
            }
        }
        if (index_l)
        {
           //更新坐标点
           center_point_l[0] = temp_l[0][0];//x
           center_point_l[1] = temp_l[0][1];//y
           for (j = 0; j < index_l; j++)
           {
              if (center_point_l[1] > temp_l[j][1])//取y小的就是高的
              {
                  center_point_l[0] = temp_l[j][0];//x
                  center_point_l[1] = temp_l[j][1];//y
              }
            }
        }
        for (i=0;i<8;i++)
        {
           if(search_filds_l[i][0]==center_point_l[0] && search_filds_l[i][1]==center_point_l[1])
            {
            dir_l[l_data_statics- 1] = i;//存储生长方向
            }
         }
        if (
              l_data_statics>2 &&
              r_data_statics>2 &&
              (((points_r[r_data_statics][0] == points_r[r_data_statics-1][0]) &&
              (points_r[r_data_statics][0] == points_r[r_data_statics-2][0]) &&
              (points_r[r_data_statics][1] == points_r[r_data_statics-1][1]) &&
              (points_r[r_data_statics][1] == points_r[r_data_statics-2][1])) ||
              ((points_l[l_data_statics][0] == points_l[l_data_statics-1][0]) &&
               (points_l[l_data_statics][1] == points_l[l_data_statics-1][1]) &&
               (points_l[l_data_statics][0] == points_l[l_data_statics-2][0]) &&
               (points_l[l_data_statics][1] == points_l[l_data_statics-2][1])))

            )

        {
            printf("三次识别同一点\r\n");//
            break;
        }


        //右边判断
        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
             temp_r[i][0] = 0;//先清零，后使用
             temp_r[i][1] = 0;//先清零，后使用
        }
        for (i = 0; i < 8; i++)
        {
           if (bin_Multi_threshold[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                                    && bin_Multi_threshold[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
           {
               temp_r[index_r][0] = search_filds_r[i][0];
               temp_r[index_r][1] = search_filds_r[i][1];
               index_r++;//索引加一
           }

         }

         if (index_r)
         {

                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

                for (i=0;i<8;i++)
               {
                    if(search_filds_r[i][0]==center_point_r[0] && search_filds_r[i][1]==center_point_r[1])
                    {
                        dir_r[r_data_statics - 1] = i;//记录生长方向
                    }
               }
         }


         //同时达到才退出，不然就是一边等待
         if (center_point_r[1]<Search_Stop_Line && center_point_r[1]<Search_Stop_Line)
         {
             return;
         }
         if (center_point_r[1]<Search_Stop_Line)
         {
             r_data_statics--;
             center_point_r[0] = points_r[r_data_statics][0];
             center_point_r[1] = points_r[r_data_statics][1];
         }
         else if (center_point_l[1]<Search_Stop_Line)
         {
             l_data_statics--;
             center_point_l[0] = points_l[l_data_statics][0];
             center_point_l[1] = points_l[l_data_statics][1];
         }
     }
}

//得到边线
void get_bianxian()
{
    uint16 j = 0;

    //左边
    for (j = 1; j < l_data_statics; j++)
    {
        if (Left_Line[points_l[j][1]]<points_l[j][0])//取中间的
        {
            Left_Line[points_l[j][1]]=points_l[j][0];
        }

    }
    for (j = 1; j < r_data_statics; j++)
    {
      if (Right_Line[points_r[j][1]]>points_r[j][0])//取中间的
      {
          Right_Line[points_r[j][1]]=points_r[j][0];
      }
    }



}


//得到左右边界的起始点
void starting_point()
{
    uint8 i;
    for (i=Sweeping_line_start;i<Search_Stop_Line;i--)
    {
        if (!Boundry_Start_Left && Left_Line[i]>border_min)
        {
            Boundry_Start_Left=i;

        }
        if (!Boundry_Start_Right && Right_Line[i]<border_max)
        {

            Boundry_Start_Right=i;
        }
        if (Boundry_Start_Right && Boundry_Start_Left)
        {
            break;
        }
    }



}


//扫线扫的是黑线
void HzMi()
{
        uint8 i;

        init();
        image_filter();//滤波
        image_draw_rectan();//填2
        get_zuibailie();
        if (get_start_point(Sweeping_line_start))//找到起点了，再执行八领域，没找到就一直找
        {

            search_l_r();
            //八邻域已结束
            // 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
            get_bianxian();

            starting_point();
            for (i = 0; i < l_data_statics; i++)
            {
                uint8 x=points_l[i][0]-2>1?points_l[i][0]-2:1;
                uint8 y=points_l[i][1];
                if (i>0&&((dir_l[i-1]==1)||(dir_l[i-1]==7)))
                {
                    x=points_l[i][0]+2;
                }

                if (i>0&&dir_l[i-1]==6)
                {
                     y=y-2<1?1:y-2;
                }
                if (i>0&&dir_l[i-1]==2)
                {
                    y=y+2;
                }
                bin_Multi_threshold[y][x]=white;
             }

             for (i = 0; i < r_data_statics; i++)
             {
                uint8 x=points_r[i][0]+2<MT9V03X_W-2?points_r[i][0]+2:MT9V03X_W-2;
                uint8 y=points_r[i][1];
                if (i>0&&((dir_r[i-1]==1)||(dir_r[i-1]==7)))
                {
                    x=points_r[i][0]-2;
                }
                if (i>0&&dir_r[i-1]==6)
                {
                    y=y-2<=1?1:y-2;
                 }
                 if (i>0&&dir_r[i-1]==2)
                 {
                     y=y+2>=MT9V03X_H-1?MT9V03X_H-1:y+2;
                 }
                 bin_Multi_threshold[y][x]=white;
              }

//             for (i = Search_Stop_Line; i < MT9V03X_H-1; i++)
//             {
//
//                 Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;//求中线
//                 bin_Multi_threshold[i][Mid_Line[i]]=black;
//             }
        }
}

















