/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cpu0_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.4
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include <Cpu0Init.h>
#include <PID.h>
#include <Platform_Types.h>
#include <seekfree_assistant.h>
#include <stdint.h>
#include <stdio.h>
#include <zf_common_clock.h>
#include <zf_common_debug.h>
#include <zf_device_ips200.h>
#include <zf_device_key.h>

#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
float p=0,i=0,d=0;
char RollShow[15]={0};
char YawShow[15]={0};
char ServoShow[15]={0};
char dynzeroShow[15]={0};
extern float gain;
extern uint8_t ServoShow1;
extern float dynzero;
int core0_main(void)
{

    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������

    Cpu0_Init();        //���ԣ�printf����λ��������֣�ʹ�����ߴ���

//    system_delay_init();
//    wireless_uart_init();
//    ips200_show_char(10,10,'a');
    key_init(10);
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

//printf("a\n");
        // �˴���д��Ҫѭ��ִ�еĴ���
/*=======================������ֵ��Σ�����ͨ��0Ϊ����===========================================*/
        seekfree_assistant_data_analysis();
        if((seekfree_assistant_parameter_update_flag[0])||(seekfree_assistant_parameter_update_flag[1])||(seekfree_assistant_parameter_update_flag[2])||(seekfree_assistant_parameter_update_flag[3])||(seekfree_assistant_parameter_update_flag[4]))
        {
            seekfree_assistant_parameter_update_flag[0]=0;
            seekfree_assistant_parameter_update_flag[1]=0;
            seekfree_assistant_parameter_update_flag[2]=0;
            seekfree_assistant_parameter_update_flag[3]=0;
            seekfree_assistant_parameter_update_flag[4]=0;
            p=seekfree_assistant_parameter[0];
            i=seekfree_assistant_parameter[1];
            d=seekfree_assistant_parameter[2];
//            Set_Servo((uint8_t)seekfree_assistant_parameter[3]);
//        MotorRun(2,1,seekfree_assistant_parameter[3]);
            gain=seekfree_assistant_parameter[3];
//                    PID_Init(&Gx,p,i,d,9000);
//            POS_PID_Init(&Roll,p,i,d,gain,9000);
//            POS_PID_Init(&Yaw,p,i,d,30,9000);
            POS_PID_Init(&Motor1,p,i,d,0.6,90);
//            printf("p=%f\n",seekfree_assistant_parameter[0]);
//            printf("i=%f\n",seekfree_assistant_parameter[1]);
//            printf("d=%f\n",seekfree_assistant_parameter[2]);
        }
/*=========================================================================================*/



        key_scanner();
        if(key_get_state (KEY_1)==KEY_SHORT_PRESS)
        {
            dynzero+=0.1;
            key_clear_state(KEY_1);
        }
        if(key_get_state (KEY_2)==KEY_SHORT_PRESS)
        {
            dynzero-=0.1;
            key_clear_state(KEY_2);
        }
//        Camera_Show();
        sprintf(RollShow,"Roll=%f",Roll.ActualValue);
        sprintf(YawShow,"Yaw=%f",Yaw.ActualValue);
        sprintf(ServoShow,"Servo=%d",ServoShow1);
        sprintf(dynzeroShow,"Dynzero=%f",dynzero);
            ips200_show_string(10,10,RollShow);
            ips200_show_string(10,30,YawShow);
            ips200_show_string(10,50,ServoShow);
            ips200_show_string(10,70,dynzeroShow);
//        printf("ok");

//        PID_Init(&Gx,p,2.62,d,9000);

//        POS_PID_Init(&Roll,p,i,d,1000,10000);
        // �˴���д��Ҫѭ��ִ�еĴ���


    }
}


#pragma section all restore


// **************************** �������� ****************************
