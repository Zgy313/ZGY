/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
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
* �ļ�����          main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
// *************************** ����Ӳ������˵�� ***************************
// ʹ����ɿƼ���������������
//      ֱ�ӽ���������ȷ�����ں��İ�ĵ������ؽӿڼ���
//
// ����С�����������ͷ ��Ӧ��������ͷ�ӿ� ��ע������
//      ģ��ܽ�            ��Ƭ���ܽ�
//      TXD                 �鿴 zf_device_ov7725.h �� OV7725_COF_UART_TX �궨��
//      RXD                 �鿴 zf_device_ov7725.h �� OV7725_COF_UART_RX �궨��
//      PCLK                �鿴 zf_device_ov7725.h �� OV7725_PCLK_PIN �궨��
//      VSY                 �鿴 zf_device_ov7725.h �� OV7725_VSYNC_PIN �궨��
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ

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
// 5.ʹ��115200�����ʡ�������ֵ�ͼ����5֡���������ڴ��ڴ��������µģ���������ͷ�ɼ����⡣
//
// 6.115200������һ���Ӵ�Լ����11.25KB���ݣ�һ��ͼ��160x120�ķֱ��ʴ�Լ��2.4KB�����ݡ�
//

// **************************** �������� ****************************
//0���������߽���Ϣ
//1����������������Ϣ��������Ϣֻ�����������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����
//2����������������Ϣ���߽���Ϣֻ�����������꣬����������ͼ���ȵõ�����ζ��ÿ���߽���һ����ֻ����һ���㣬һ����˵������������ʹ������
//3����������������Ϣ���߽���Ϣ���к��������꣬��ζ�������ָ��ÿ����ĺ������꣬���ߵ�����Ҳ���Դ��ڻ���С��ͼ��ĸ߶ȣ�ͨ����˵������������ͼ��ĸ߶ȣ�һ���������㷨���ҳ���������
//4��û��ͼ����Ϣ������������������Ϣ��������Ϣֻ�����������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ���㣬�����ķ�ʽ���Լ���Ľ��ʹ����������
#define INCLUDE_BOUNDARY_TYPE   3

// �߽�ĵ�����Զ����ͼ��߶ȣ����ڱ����������
#define BOUNDARY_NUM            (OV7725_H * 3 / 2)

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[OV7725_H], x2_boundary[OV7725_H], x3_boundary[OV7725_H];
uint8 y1_boundary[OV7725_W], y2_boundary[OV7725_W], y3_boundary[OV7725_W];

// ͼ�񱸷����飬�ڷ���ǰ��ͼ�񱸷��ٽ��з��ͣ��������Ա���ͼ�����˺�ѵ�����
uint8 image_copy[OV7725_H][OV7725_W/8];

#define LED1                    (B9 )

int main (void)
{
    clock_init(SYSTEM_CLOCK_600M);                                              // ����ɾ��
    debug_init();                                                               // ���Զ˿ڳ�ʼ��
    
    // �����������ʹ��DEBUG���ڽ����շ�
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
#if(0 != INCLUDE_BOUNDARY_TYPE)
    int32 i=0;
#endif

#if(3 <= INCLUDE_BOUNDARY_TYPE)
    int32 j=0;
#endif

    gpio_init(LED1, GPO, 0, FAST_GPO_PUSH_PULL);                                // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ

    // �˴���д�û����� ���������ʼ�������
    while(1)
    {
        if(ov7725_init())
            gpio_toggle_level(LED1);                                            // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
        else
            break;
        system_delay_ms(500);                                                   // ���Ʊ�ʾ�쳣
    }

#if(0 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(������ԭʼͼ����Ϣ)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_OV7725_BIN, image_copy[0], OV7725_W, OV7725_H);

#elif(1 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣֻ���к������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����)
    // �Ա߽�����д������
    for(i = 0; i < OV7725_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / OV7725_H;
        x2_boundary[i] = OV7725_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * i / OV7725_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_OV7725_BIN, image_copy[0], OV7725_W, OV7725_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, OV7725_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


#elif(2 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣֻ�����������꣬����������ͼ���ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����)
    // ͨ��������������ʹ������
    // �Ա߽�����д������
    for(i = 0; i < OV7725_W; i++)
    {
        y1_boundary[i] = i * OV7725_H / OV7725_W;
        y2_boundary[i] = OV7725_H / 2;
        y3_boundary[i] = (OV7725_W - i) * OV7725_H / OV7725_W;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_OV7725_BIN, image_copy[0], OV7725_W, OV7725_H);
    seekfree_assistant_camera_boundary_config(Y_BOUNDARY, OV7725_W, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);


#elif(3 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣ���к���������)
    // �����ķ�ʽ����ʵ�ֶ����л���ı߽���ʾ
    j = 0;
    for(i = OV7725_H - 1; i >= OV7725_H / 2; i--)
    {
        // ֱ�߲���
        xy_x1_boundary[j] = 34;
        xy_y1_boundary[j] = i;

        xy_x2_boundary[j] = 47;
        xy_y2_boundary[j] = i;

        xy_x3_boundary[j] = 60;
        xy_y3_boundary[j] = i;
        j++;
    }

    for(i = OV7725_H / 2 - 1; i >= 0; i--)
    {
        // ֱ�������������
        xy_x1_boundary[j] = 34 + (OV7725_H / 2 - i) * (OV7725_W / 2 - 34) / (OV7725_H / 2);
        xy_y1_boundary[j] = i;

        xy_x2_boundary[j] = 47 + (OV7725_H / 2 - i) * (OV7725_W / 2 - 47) / (OV7725_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;

        xy_x3_boundary[j] = 60 + (OV7725_H / 2 - i) * (OV7725_W / 2 - 60) / (OV7725_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }

    for(i = 0; i < OV7725_H / 2; i++)
    {
        // ���䲿��
        xy_x1_boundary[j] = OV7725_W / 2 + i * (138 - OV7725_W / 2) / (OV7725_H / 2);
        xy_y1_boundary[j] = i;

        xy_x2_boundary[j] = OV7725_W / 2 + i * (133 - OV7725_W / 2) / (OV7725_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;

        xy_x3_boundary[j] = OV7725_W / 2 + i * (128 - OV7725_W / 2) / (OV7725_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_OV7725_BIN, image_copy[0], OV7725_W, OV7725_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);


#elif(4 == INCLUDE_BOUNDARY_TYPE)
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣֻ���к������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����)
    // �Ա߽�����д������
    for(i = 0; i < OV7725_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / OV7725_H;
        x2_boundary[i] = OV7725_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * i / OV7725_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_OV7725_BIN, NULL, OV7725_W, OV7725_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, OV7725_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


#endif


    // �˴���д�û����� ���������ʼ�������

    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

        if(ov7725_finish_flag)
        {
            ov7725_finish_flag = 0;

            // �ڷ���ǰ��ͼ�񱸷��ٽ��з��ͣ��������Ա���ͼ�����˺�ѵ�����
            memcpy(image_copy[0], ov7725_image_binary[0], OV7725_IMAGE_SIZE);

            // ����ͼ��
            seekfree_assistant_camera_send();
        }

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}
