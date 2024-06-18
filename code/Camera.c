#include <Platform_Types.h>
#include <stdint.h>
#include <zf_device_ips200.h>
#include <zf_device_mt9v03x.h>
#include "Camera.h"


//����ͷ��LCD��ʾ��ʼ��
void Camera_Show_Init(void)
{
    mt9v03x_init();
    ips200_init(IPS200_TYPE_SPI);

}

//������ͷ������ʾ��LCD��
//��ѭ���е��ø�������ͷ����

//void Camera_Show(void)
//{
//
//    if(mt9v03x_finish_flag)
//    {
//        uint8_t OTSU_Threshold = otsuThreshold(mt9v03x_image[0]);//�����ֵ
//        //��ʾ����λ����
//
////        seekfree_assistant_camera_send();
//
//        //��ʾ��LCD��
//        //�Ҷ�ͼ��
////        ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//        //��ֵ��ͼ��
//        ips200_show_gray_image(30, 10, mt9v03x_image[0], MT9V03X_W, MT9V03X_H,MT9V03X_W, MT9V03X_H, OTSU_Threshold );
//        mt9v03x_finish_flag=0;
//    }
//
//}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ٴ��
//  @return     uint8
//  @since      v1.1
//  Sample usage:   OTSU_Threshold = otsuThreshold(mt9v03x_image_dvp[0]);//�����ֵ
//-------------------------------------------------------------------------------------------------------------------
//uint8 otsuThreshold(uint8 *image)   //ע�������ֵ��һ��Ҫ��ԭͼ��
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
//    uint8* data = image;  //ָ���������ݵ�ָ��
//    for (i = 0; i < GrayScale; i++)
//    {
//        pixelCount[i] = 0;
//        pixelPro[i] = 0;
//    }
//
//    uint32 gray_sum=0;
//    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
//    for (i = 0; i < height; i+=2)
//    {
//        for (j = 0; j < width; j+=2)
//        {
//            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
//            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
//            if(data[i * width + j]>Pixel_Max)   Pixel_Max=data[i * width + j];
//            if(data[i * width + j]<Pixel_Min)   Pixel_Min=data[i * width + j];
//        }
//    }
//
//    //����ÿ������ֵ�ĵ�������ͼ���еı���
//
//    for (i = Pixel_Min; i < Pixel_Max; i++)
//    {
//        pixelPro[i] = (float)pixelCount[i] / pixelSum;
//
//    }
//
//    //�����Ҷȼ�[0,255]
//    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
//
//    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
//    for (j = Pixel_Min; j < Pixel_Max; j++)
//    {
//
//        w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
//        u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ
//
//        w1=1-w0;
//        u1tmp=gray_sum/pixelSum-u0tmp;
//
//        u0 = u0tmp / w0;              //����ƽ���Ҷ�
//        u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
//        u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
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
