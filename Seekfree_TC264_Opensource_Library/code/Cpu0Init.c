#include <Camera.h>
#include <seekfree_assistant_interface.h>
#include <stdint.h>
#include <zf_common_clock.h>
#include <zf_device_mt9v03x.h>
#include "Cpu0Init.h"

uint8_t x1[MT9V03X_H];
uint8_t x2[MT9V03X_H];
uint8_t x3[MT9V03X_H];
//CPU0用户函数的初始化，大多是中断的初始化
void Cpu0_Init(void)
{

    //这里还需要一个外部中断初始化，用于触发测距模块

    //摄像头初始化
    Camera_Show_Init();//将摄像头和屏幕显示都初始化


//    seekfree_assistant_camera_config(X_BOUNDARY, MT9V03X_H, x1, x2, x3, NULL, NULL, NULL);
//    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,mt9v03x_image,MT9V03X_W,MT9V03X_H);

    seekfree_assistant_interface_init (SEEKFREE_ASSISTANT_WIRELESS_UART);
//编码器测速以及其超时检测初始化
//    SpeedCal_Init(&speed1,P15_5,P15_3);
//    SpeedCal_Init(&speed2,P15_4,P15_2);
    cpu_wait_event_ready();
//    pit_ms_init(CCU60_CH1,20);


}
