#include "zf_common_headfile.h"
#include "dajinfa.h"
#include "saoxian.h"


uint8 Left_Line[MT9V03X_H]; //���������
uint8 Right_Line[MT9V03X_H];//�ұ�������

uint8 Mid_Line[MT9V03X_H];  //��������
uint8 White_Column[MT9V03X_W];//ÿ�а��г���
uint8 Search_Stop_Line;     //������ֹ��

uint16 l_data_statics;//ͳ�����
uint16 r_data_statics;//ͳ���ұ�


uint8 points_l[max_length][2] ;//���� x,y
uint8 points_r[max_length][2];//���� x,y
uint16 dir_r[max_length];//�����洢�ұ���������
uint16 dir_l[max_length];//�����洢�����������
uint16 l_data_statics;//ͳ������ҵ���ĸ���
uint16 r_data_statics;//ͳ���ұ��ҵ���ĸ���

uint8 Boundry_Start_Left;   //���ұ߽���ʼ��
uint8 Boundry_Start_Right;

uint8 Sweeping_line_start=MT9V03X_H-2;//ɨ�߿�ʼ��
uint8 Standard_Road_Wide[MT9V03X_H];//��׼��������


void init()
{
    uint8 i;
    for (i=0;i<MT9V03X_H;i++)
    {
        Left_Line[i]=border_min;
        Right_Line[i]=border_max;
        Mid_Line[i]=0;
        //���ұ���Ĭ�Ϻͺڿ��غ�
    }
    for (i=0;i<MT9V03X_W;i++)
    {
        White_Column[i]=0;//��ʼ�������
    }

    Search_Stop_Line=0;     //������ֹ��
    l_data_statics=0;//ͳ�����
    r_data_statics=0;//ͳ���ұ�


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
    l_data_statics=0;//ͳ������ҵ���ĸ���
    r_data_statics=0;//ͳ���ұ��ҵ���ĸ���

    Boundry_Start_Left=0;   //���ұ߽���ʼ��
    Boundry_Start_Right=0;  //��һ���Ƕ��ߵ�,����߽���ʼ��
}

//�������ͺ͸�ʴ����ֵ����
#define threshold_max   255*5//�˲����ɸ����Լ����������
#define threshold_min   255*2//�˲����ɸ����Լ����������
void image_filter()//��̬ѧ�˲�������˵�������ͺ͸�ʴ��˼��
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < MT9V03X_H - 1; i++)
    {
        for (j = 1; j < (MT9V03X_W - 1); j++)
        {
            //ͳ�ư˸����������ֵ
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



//��ͼ��һ���ڿ�
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


//�õ������
void get_zuibailie()
{
        int16 i;
        uint8 j;
        uint8 start_column=20;//����е���������
        uint8 end_column=MT9V03X_W-20;
        //�����ң��������ϣ�����ȫͼ��¼��Χ�ڵ�ÿһ�а׵�����
        for (j =start_column; j<=end_column; j++)
        {
             for (i = Sweeping_line_start; i >= 2; i--)//��ɨ����ʼ��ɨ����ĺڿ�
             {
                    if(bin_Multi_threshold[i][j] == 0)
                        break;
                    else
                        White_Column[j]++;
              }
        }

        //����������������
        Search_Stop_Line =0;
        for(j=start_column;j<=end_column;j++)
        {
             if (Search_Stop_Line < White_Column[j])//�������һ��
             {
                 Search_Stop_Line = White_Column[j];//0�ǰ��г���

             }
        }
        Search_Stop_Line=MT9V03X_H-Search_Stop_Line;
}


//Ѱ�������߽�ı߽����Ϊ������ѭ������ʼ��
uint8 start_point_l[2] ;//�������x��yֵ
uint8 start_point_l_r[2];//�ܵ��ƶ������ߵ�һ��
uint8 start_point_r[2];//�ұ�����x��yֵ
uint8 start_point_r_l[2];
uint8 get_start_point(uint8 start_row)
{
    uint8 i;
    //����
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_l_r[0]=0;
    start_point_l_r[1]=0;

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

    start_point_r_l[0]=0;
    start_point_r_l[1]=0;

    //���м�����ߣ��������
    for (i = MT9V03X_W / 2; i > 0; i--)
    {

        if (bin_Multi_threshold[start_row][i] == 0 && bin_Multi_threshold[start_row][i - 1] == 255)
        {
            start_point_l_r[0]=i;
            start_point_l_r[1]=start_row;
        }
        if (bin_Multi_threshold[start_row][i] == 255 && bin_Multi_threshold[start_row][i - 1] == 0)//�������߾��Ǻ�
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




//������
void search_l_r()
{

    uint16 i = 0, j = 0;
    //��߱���
    uint8 search_filds_l[8][2] = { {  0 } };//�����洢���ĵ���Χ�İ˸��������
    uint8 index_l = 0;//�����±�
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };//���������

    //����˸�����
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}};
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�����˳ʱ��

    //�ұ߱���
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//���������
    uint8 index_r = 0;//�����±�
    uint8 temp_r[8][2] = { {  0 } };

    //����˸�����
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}};
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�������ʱ��

    center_point_l[0] = start_point_l[0];//x
    center_point_l[1] = start_point_l[1];//y
    center_point_r[0] = start_point_r[0];//x
    center_point_r[1] = start_point_r[1];//y

    uint16 break_flag=max_length;
    //��������ѭ��
    while (break_flag--)
    {

        //���
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }//��¼��Χ�˸��������

        //�����������䵽�Ѿ��ҵ��ĵ���
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//������һ

        //�ұ�
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        r_data_statics++;//������һ


        index_l = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//�����㣬��ʹ��
            temp_l[i][1] = 0;//�����㣬��ʹ��
        }

        //����ж�
        for (i = 0; i < 8; i++)
        {
            //image[y][x]ϸ��
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
           //���������
           center_point_l[0] = temp_l[0][0];//x
           center_point_l[1] = temp_l[0][1];//y
           for (j = 0; j < index_l; j++)
           {
              if (center_point_l[1] > temp_l[j][1])//ȡyС�ľ��Ǹߵ�
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
            dir_l[l_data_statics- 1] = i;//�洢��������
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
            printf("����ʶ��ͬһ��\r\n");//
            break;
        }


        //�ұ��ж�
        index_r = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
             temp_r[i][0] = 0;//�����㣬��ʹ��
             temp_r[i][1] = 0;//�����㣬��ʹ��
        }
        for (i = 0; i < 8; i++)
        {
           if (bin_Multi_threshold[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                                    && bin_Multi_threshold[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
           {
               temp_r[index_r][0] = search_filds_r[i][0];
               temp_r[index_r][1] = search_filds_r[i][1];
               index_r++;//������һ
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
                        dir_r[r_data_statics - 1] = i;//��¼��������
                    }
               }
         }


         //ͬʱ�ﵽ���˳�����Ȼ����һ�ߵȴ�
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

//�õ�����
void get_bianxian()
{
    uint16 j = 0;

    //���
    for (j = 1; j < l_data_statics; j++)
    {
        if (Left_Line[points_l[j][1]]<points_l[j][0])//ȡ�м��
        {
            Left_Line[points_l[j][1]]=points_l[j][0];
        }

    }
    for (j = 1; j < r_data_statics; j++)
    {
      if (Right_Line[points_r[j][1]]>points_r[j][0])//ȡ�м��
      {
          Right_Line[points_r[j][1]]=points_r[j][0];
      }
    }



}


//�õ����ұ߽����ʼ��
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


//ɨ��ɨ���Ǻ���
void HzMi()
{
        uint8 i;

        init();
        image_filter();//�˲�
        image_draw_rectan();//��2
        get_zuibailie();
        if (get_start_point(Sweeping_line_start))//�ҵ�����ˣ���ִ�а�����û�ҵ���һֱ��
        {

            search_l_r();
            //�������ѽ���
            // ����ȡ�ı߽�������ȡ���� �� ��������������õı���
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
//                 Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;//������
//                 bin_Multi_threshold[i][Mid_Line[i]]=black;
//             }
        }
}

















