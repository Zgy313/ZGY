#ifndef SAOXIAN_H
#define SAOXIAN_H

//边框填2，又边线得到的是黑线，所以默认1
#define border_max  MT9V03X_W-2 //边界最大值
#define border_min  1   //边界最小值
#define max_length MT9V03X_H*2 //边界最大存储数量


extern uint8  Standard_Road_Wide[MT9V03X_H];//标准赛宽数组
//要自己去在测

extern uint8 Left_Line[MT9V03X_H]; //左边线数组
extern uint8 Right_Line[MT9V03X_H];//右边线数组

extern uint8 Mid_Line[MT9V03X_H];  //中线数组
extern uint8 White_Column[MT9V03X_W];//每列白列长度
extern uint8 Search_Stop_Line;     //搜索截止行

extern uint16 l_data_statics;//统计左边
extern uint16 r_data_statics;//统计右边


extern uint8 points_l[max_length][2] ;//左线 x,y
extern uint8 points_r[max_length][2];//右线 x,y
extern uint16 dir_r[max_length];//用来存储右边生长方向
extern uint16 dir_l[max_length];//用来存储左边生长方向
extern uint16 data_stastics_l;//统计左边找到点的个数
extern uint16 data_stastics_r;//统计右边找到点的个数
extern uint8 Boundry_Start_Left;   //左右边界起始点
extern uint8 Boundry_Start_Right;  //第一个非丢线点,常规边界起始点

extern uint8 Boundry_Start_Left;   //左右边界起始点
extern uint8 Boundry_Start_Right;
extern uint8 Sweeping_line_start;
void HzMi(void);

#endif

