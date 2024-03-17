#include <seekfree_assistant_interface.h>
#include <zf_common_clock.h>
#include <zf_device_ips200.h>
#include "Cpu0Init.h"


//CPU0用户函数的初始化，大多是中断的初始化
void Cpu0_Init(void)
{

    //这里还需要一个外部中断初始化，用于触发测距模块

    //摄像头初始化
//    Camera_Show_Init();//将摄像头和屏幕显示都初始化


//    seekfree_assistant_camera_config(X_BOUNDARY, MT9V03X_H, x1, x2, x3, NULL, NULL, NULL);
//    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,mt9v03x_image,MT9V03X_W,MT9V03X_H);
    ips200_init(IPS200_TYPE_PARALLEL8);
    seekfree_assistant_interface_init (SEEKFREE_ASSISTANT_WIRELESS_UART);

    cpu_wait_event_ready();



}
