#ifndef SAOXIAN_H
#define SAOXIAN_H

//�߿���2���ֱ��ߵõ����Ǻ��ߣ�����Ĭ��1
#define border_max  MT9V03X_W-2 //�߽����ֵ
#define border_min  1   //�߽���Сֵ
#define max_length MT9V03X_H*2 //�߽����洢����


extern uint8  Standard_Road_Wide[MT9V03X_H];//��׼��������
//Ҫ�Լ�ȥ�ڲ�

extern uint8 Left_Line[MT9V03X_H]; //���������
extern uint8 Right_Line[MT9V03X_H];//�ұ�������

extern uint8 Mid_Line[MT9V03X_H];  //��������
extern uint8 White_Column[MT9V03X_W];//ÿ�а��г���
extern uint8 Search_Stop_Line;     //������ֹ��

extern uint16 l_data_statics;//ͳ�����
extern uint16 r_data_statics;//ͳ���ұ�


extern uint8 points_l[max_length][2] ;//���� x,y
extern uint8 points_r[max_length][2];//���� x,y
extern uint16 dir_r[max_length];//�����洢�ұ���������
extern uint16 dir_l[max_length];//�����洢�����������
extern uint16 data_stastics_l;//ͳ������ҵ���ĸ���
extern uint16 data_stastics_r;//ͳ���ұ��ҵ���ĸ���
extern uint8 Boundry_Start_Left;   //���ұ߽���ʼ��
extern uint8 Boundry_Start_Right;  //��һ���Ƕ��ߵ�,����߽���ʼ��

extern uint8 Boundry_Start_Left;   //���ұ߽���ʼ��
extern uint8 Boundry_Start_Right;
extern uint8 Sweeping_line_start;
void HzMi(void);

#endif

