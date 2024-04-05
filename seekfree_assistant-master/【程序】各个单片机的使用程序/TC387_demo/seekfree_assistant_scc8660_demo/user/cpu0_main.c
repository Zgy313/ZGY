/*********************************************************************************************************************
* TC387 Opensourec Library ����TC387 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC387 ��Դ���һ����
*
* TC387 ��Դ�� ��������
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
* ����ƽ̨          TC387QP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-11-04       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// *************************** ����Ӳ������˵�� ***************************
// ʹ����ɿƼ���������������
//      ֱ�ӽ���������ȷ�����ں��İ�ĵ������ؽӿڼ���
//
// ���� ���������ͷ
//      ģ��ܽ�            ��Ƭ���ܽ�
//      TXD                 �鿴 zf_device_scc8660.h �� SCC8660_COF_UART_TX        �궨��
//      RXD                 �鿴 zf_device_scc8660.h �� SCC8660_COF_UART_RX        �궨��
//      D0                  �鿴 zf_device_scc8660.h �� SCC8660_D0_PIN             �궨��
//      D1                  �鿴 zf_device_scc8660.h �� SCC8660_D1_PIN             �궨��
//      D2                  �鿴 zf_device_scc8660.h �� SCC8660_D2_PIN             �궨��
//      D3                  �鿴 zf_device_scc8660.h �� SCC8660_D3_PIN             �궨��
//      D4                  �鿴 zf_device_scc8660.h �� SCC8660_D4_PIN             �궨��
//      D5                  �鿴 zf_device_scc8660.h �� SCC8660_D5_PIN             �궨��
//      D6                  �鿴 zf_device_scc8660.h �� SCC8660_D6_PIN             �궨��
//      D7                  �鿴 zf_device_scc8660.h �� SCC8660_D7_PIN             �궨��
//      PCLK                �鿴 zf_device_scc8660.h �� SCC8660_PCLK_PIN           �궨��
//      VSYNC               �鿴 zf_device_scc8660.h �� SCC8660_VSY_PIN            �궨��
//      HSYNC               �鿴 zf_device_scc8660.h �� SCC8660_HERF_PIN           �궨��
//  ------------------------------------

// *************************** ����ʹ�ò���˵�� ***************************
// 1.����Ӳ������˵�����Ӻ�ģ�飬ʹ�õ�Դ����(����������ᵼ��ģ���ѹ����)
//
// 2.�������̵���Ƭ���У�����ɴ������֡�
//
// 3.����ɴ��������У�ѡ��ͼ���䡣
//
// 4.ѡ����������Ӧ�Ĵ��ںţ�������(Ĭ��115200)���������
//
// 5.�ȴ������ӣ�ͼ��ͻ���ʾ����ɴ��������С�


// *************************** ���̲���˵�� ***************************
// 1.�����̻�ͨ�� Debug �������������Ϣ ����ؽӺõ��Դ����Ա��ȡ������Ϣ
//
// 2.���Ӻ�ģ��ͺ��İ�󣨾���ʹ��������������Ա��⹩�粻������⣩ ��¼������ ���¸�λ�����ʼ����
//
// 3.���ģ��δ��������ʼ�� ��ͨ�� DEBUG �������δ�ܳɹ���ʼ����ԭ�� ������᳢�����³�ʼ�� һ����������Ի�ɹ�
//
// 4.���һֱ�� Debug ����������� ����Ҫ��鱨������ ���鿴���ļ��·��ĳ��������б�����Ų�
//
// 5.ʹ��115200�����ʡ�������ֵ�ͼ����������һ֡���������ڴ��ڴ��������µģ���������ͷ�ɼ����⡣
//
// 6.115200������һ���Ӵ�Լ����11.25KB���ݣ�һ��ͼ��188x120�ķֱ��ʴ�Լ��22KB�����ݡ�
//

// **************************** �������� ****************************
//0���������߽���Ϣ
//1����������������Ϣ��������Ϣֻ�����������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����
//2����������������Ϣ���߽���Ϣֻ�����������꣬����������ͼ���ȵõ�����ζ��ÿ���߽���һ����ֻ����һ���㣬һ����˵������������ʹ������
//3����������������Ϣ���߽���Ϣ���к��������꣬��ζ�������ָ��ÿ����ĺ������꣬���ߵ�����Ҳ���Դ��ڻ���С��ͼ��ĸ߶ȣ�ͨ����˵������������ͼ��ĸ߶ȣ�һ���������㷨���ҳ���������
//4��û��ͼ����Ϣ������������������Ϣ��������Ϣֻ�����������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ���㣬�����ķ�ʽ���Լ���Ľ��ʹ����������
#define INCLUDE_BOUNDARY_TYPE   3

// �߽�ĵ�����Զ����ͼ��߶ȣ����ڱ����������
#define BOUNDARY_NUM            (SCC8660_H * 3 / 2)

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[SCC8660_H], x2_boundary[SCC8660_H], x3_boundary[SCC8660_H];
uint8 y1_boundary[SCC8660_W], y2_boundary[SCC8660_W], y3_boundary[SCC8660_W];

// ͼ�񱸷����飬�ڷ���ǰ��ͼ�񱸷��ٽ��з��ͣ��������Ա���ͼ�����˺�ѵ�����
uint16 image_copy[SCC8660_H][SCC8660_W];

#define LED1                    (P20_9 )

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������

    // �����������ʹ��DEBUG���ڽ����շ�
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

#if(0 != INCLUDE_BOUNDARY_TYPE)
    int32 i=0;
#endif

#if(3 == INCLUDE_BOUNDARY_TYPE)
    int32 j=0;
#endif

    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ

    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        if(scc8660_init())
            gpio_toggle_level(LED1);                                            // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
        else
            break;
        system_delay_ms(500);                                                  // ���Ʊ�ʾ�쳣
    }

#if(0 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(������ԭʼͼ����Ϣ)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, scc8660_image[0], SCC8660_W, SCC8660_H);

#elif(1 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣֻ���к������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����)
    // �Ա߽�����д������
    for(i = 0; i < SCC8660_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * (uint8)i / SCC8660_H;
        x2_boundary[i] = SCC8660_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * (uint8)i / SCC8660_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, image_copy[0], SCC8660_W, SCC8660_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, SCC8660_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


#elif(2 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣֻ�����������꣬����������ͼ���ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����)
    // ͨ��������������ʹ������
    // �Ա߽�����д������
    for(i = 0; i < SCC8660_W; i++)
    {
        y1_boundary[i] = (uint8)i * SCC8660_H / SCC8660_W;
        y2_boundary[i] = SCC8660_H / 2;
        y3_boundary[i] = (SCC8660_W - (uint8)i) * SCC8660_H / SCC8660_W;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, image_copy[0], SCC8660_W, SCC8660_H);
    seekfree_assistant_camera_boundary_config(Y_BOUNDARY, SCC8660_W, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);


#elif(3 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣ���к���������)
    // �����ķ�ʽ����ʵ�ֶ����л���ı߽���ʾ
    j = 0;
    for(i = SCC8660_H - 1; i >= SCC8660_H / 2; i--)
    {
        // ֱ�߲���
        xy_x1_boundary[j] = 34;
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = 47;
        xy_y2_boundary[j] = (uint8)i;

        xy_x3_boundary[j] = 60;
        xy_y3_boundary[j] = (uint8)i;
        j++;
    }

    for(i = SCC8660_H / 2 - 1; i >= 0; i--)
    {
        // ֱ�������������
        xy_x1_boundary[j] = 34 + (SCC8660_H / 2 - (uint8)i) * (SCC8660_W / 2 - 34) / (SCC8660_H / 2);
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = 47 + (SCC8660_H / 2 - (uint8)i) * (SCC8660_W / 2 - 47) / (SCC8660_H / 2);
        xy_y2_boundary[j] = 15 + (uint8)i * 3 / 4;

        xy_x3_boundary[j] = 60 + (SCC8660_H / 2 - (uint8)i) * (SCC8660_W / 2 - 60) / (SCC8660_H / 2);
        xy_y3_boundary[j] = 30 + (uint8)i / 2;
        j++;
    }

    for(i = 0; i < SCC8660_H / 2; i++)
    {
        // ���䲿��
        xy_x1_boundary[j] = SCC8660_W / 2 + (uint8)i * (138 - SCC8660_W / 2) / (SCC8660_H / 2);
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = SCC8660_W / 2 + (uint8)i * (133 - SCC8660_W / 2) / (SCC8660_H / 2);
        xy_y2_boundary[j] = 15 + (uint8)i * 3 / 4;

        xy_x3_boundary[j] = SCC8660_W / 2 + (uint8)i * (128 - SCC8660_W / 2) / (SCC8660_H / 2);
        xy_y3_boundary[j] = 30 + (uint8)i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, image_copy[0], SCC8660_W, SCC8660_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);


#elif(4 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣֻ���к������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����)
    // �Ա߽�����д������
    for(i = 0; i < SCC8660_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * (uint8)i / SCC8660_H;
        x2_boundary[i] = SCC8660_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * (uint8)i / SCC8660_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, NULL, SCC8660_W, SCC8660_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, SCC8660_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


#endif

    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

        if(scc8660_finish_flag)
        {
            scc8660_finish_flag = 0;

            memcpy(image_copy[0], scc8660_image[0], SCC8660_IMAGE_SIZE);
            // ����ͼ��
            seekfree_assistant_camera_send();
        }

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore


// **************************** �������� ****************************
