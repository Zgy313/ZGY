#include <stdio.h>
#include <zf_device_imu660ra.h>
#include <zf_driver_delay.h>
#include "MotorRun.h"


u8 DistanceDetect=0;

/*=========================注意：不要在该函数中写延时函数！！！=================*/
//摩托车运行函数（放在CPU1中）
void Motor_Run(void)
{

//    if(PIDCalFlag==1)
//    {
//        printf("ok\n");
//      PID_Realize();//写的有问题
//        PIDCalFlag=0;
//    }

    //由摄像头设置标志位触发测距
//    if(DistanceDetect==1)
//    {
//        dl1b_get_distance();
//        if(dl1b_finsh_flag==1)
//        {
//            if(dl1b_distance_mm<=200)
//            {
//                //靠近障碍物做相应的处理
//
//                dl1b_finsh_flag=0;
////                DistanceDetect=0;//清楚标志位
//            }else dl1b_finsh_flag=0;
//        }
//    }


    //陀螺仪调试
//    imu660ra_get_gyro();
//    imu660ra_get_acc();
//    ips200_show_string(0,10,"GX:");
//    ips200_show_int(10,10,imu660ra_gyro_x,4);
//    ips200_show_string(0,30,"GY:");
//    ips200_show_int(10,30,imu660ra_gyro_y,4);
//    ips200_show_string(0,50,"GZ:");
//    ips200_show_int(10,50,imu660ra_gyro_z,4);
//    ips200_show_string(0,70,"AX:");
//    ips200_show_int(10,70,imu660ra_acc_x,4);
//    ips200_show_string(0,90,"AX:");
//    ips200_show_int(10,90,imu660ra_acc_y,4);
//    ips200_show_string(0,110,"AX:");
//    ips200_show_int(10,110,imu660ra_acc_z,4);

//    printf("GX=%d\n",imu660ra_gyro_x);
//    printf("GY=%d\n",imu660ra_gyro_y);
//    printf("GZ=%d\n",imu660ra_gyro_z);
//    printf("AX=%d\n",imu660ra_acc_x);
//    printf("AY=%d\n",imu660ra_acc_y);
//    printf("AZ=%d\n",imu660ra_acc_z);
//    system_delay_ms(1000);


}
