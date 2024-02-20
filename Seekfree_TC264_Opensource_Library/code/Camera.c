#include <seekfree_assistant.h>
#include <zf_device_mt9v03x.h>
#include "Camera.h"


//摄像头在LCD显示初始化
void Camera_Show_Init(void)
{
    mt9v03x_init();
//    ips200_init(IPS200_TYPE_SPI);

}

//将摄像头画面显示在LCD上
//在循环中调用更新摄像头画面

void Camera_Show(void)
{
    if(mt9v03x_finish_flag)
    {
        //显示在LCD上
//        ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
        //显示在上位机上

        seekfree_assistant_camera_send();

        mt9v03x_finish_flag=0;
    }

}

