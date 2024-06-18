#include <Platform_Types.h>
#include <stdint.h>
#include <zf_device_ips200.h>
#include <zf_device_mt9v03x.h>
#include "Camera.h"


//摄像头在LCD显示初始化
void Camera_Show_Init(void)
{
    mt9v03x_init();
    ips200_init(IPS200_TYPE_SPI);

}

//将摄像头画面显示在LCD上
//在循环中调用更新摄像头画面

//void Camera_Show(void)
//{
//
//    if(mt9v03x_finish_flag)
//    {
//        uint8_t OTSU_Threshold = otsuThreshold(mt9v03x_image[0]);//大津法阈值
//        //显示在上位机上
//
////        seekfree_assistant_camera_send();
//
//        //显示在LCD上
//        //灰度图像
////        ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//        //二值化图像
//        ips200_show_gray_image(30, 10, mt9v03x_image[0], MT9V03X_W, MT9V03X_H,MT9V03X_W, MT9V03X_H, OTSU_Threshold );
//        mt9v03x_finish_flag=0;
//    }
//
//}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      快速大津
//  @return     uint8
//  @since      v1.1
//  Sample usage:   OTSU_Threshold = otsuThreshold(mt9v03x_image_dvp[0]);//大津法阈值
//-------------------------------------------------------------------------------------------------------------------
//uint8 otsuThreshold(uint8 *image)   //注意计算阈值的一定要是原图像
//{
//#define GrayScale 256
//    static uint8_t last_threshold=0;
//    int Pixel_Max=0;
//    int Pixel_Min=255;
//    uint16 width = MT9V03X_W;
//    uint16 height = MT9V03X_H;
//    int pixelCount[GrayScale];
//    float pixelPro[GrayScale];
//    int i, j, pixelSum = width * height/4;
//    uint8 threshold = 0;
//    uint8* data = image;  //指向像素数据的指针
//    for (i = 0; i < GrayScale; i++)
//    {
//        pixelCount[i] = 0;
//        pixelPro[i] = 0;
//    }
//
//    uint32 gray_sum=0;
//    //统计灰度级中每个像素在整幅图像中的个数
//    for (i = 0; i < height; i+=2)
//    {
//        for (j = 0; j < width; j+=2)
//        {
//            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
//            gray_sum+=(int)data[i * width + j];       //灰度值总和
//            if(data[i * width + j]>Pixel_Max)   Pixel_Max=data[i * width + j];
//            if(data[i * width + j]<Pixel_Min)   Pixel_Min=data[i * width + j];
//        }
//    }
//
//    //计算每个像素值的点在整幅图像中的比例
//
//    for (i = Pixel_Min; i < Pixel_Max; i++)
//    {
//        pixelPro[i] = (float)pixelCount[i] / pixelSum;
//
//    }
//
//    //遍历灰度级[0,255]
//    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
//
//    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
//    for (j = Pixel_Min; j < Pixel_Max; j++)
//    {
//
//        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
//        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
//
//        w1=1-w0;
//        u1tmp=gray_sum/pixelSum-u0tmp;
//
//        u0 = u0tmp / w0;              //背景平均灰度
//        u1 = u1tmp / w1;              //前景平均灰度
//        u = u0tmp + u1tmp;            //全局平均灰度
//        deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;
//        if (deltaTmp > deltaMax)
//        {
//            deltaMax = deltaTmp;
//            threshold = (uint8_t)j;
//        }
//        if (deltaTmp < deltaMax)
//        {
//            break;
//        }
//
//    }
//
//    if(threshold>90 && threshold<130)
//        last_threshold = threshold;
//    else
//        threshold = last_threshold;
//
//    return threshold;
//}
