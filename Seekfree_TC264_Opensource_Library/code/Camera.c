#include <seekfree_assistant.h>
#include <zf_device_mt9v03x.h>
#include "Camera.h"


//����ͷ��LCD��ʾ��ʼ��
void Camera_Show_Init(void)
{
    mt9v03x_init();
//    ips200_init(IPS200_TYPE_SPI);

}

//������ͷ������ʾ��LCD��
//��ѭ���е��ø�������ͷ����

void Camera_Show(void)
{
    if(mt9v03x_finish_flag)
    {
        //��ʾ��LCD��
//        ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
        //��ʾ����λ����

        seekfree_assistant_camera_send();

        mt9v03x_finish_flag=0;
    }

}

