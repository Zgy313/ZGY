#include "zf_common_headfile.h"
#include <math.h>

uint8 original_image[MT9V03X_H][MT9V03X_W];
uint8 Difference_image[MT9V03X_H][MT9V03X_W];


void get_original()
{
    uint8 i, j;
    for (i = 0; i < MT9V03X_H; i ++)
    {
      for (j = 0; j <MT9V03X_W; j ++)
      {
             original_image[i][j] = mt9v03x_image[i][j];
      }
    }
}

uint8 otsuThreshold(uint8 tmImage[MT9V03X_H][MT9V03X_W]);

void Difference_ratio_and()
{

    uint8 i, j;
    get_original();
    uint8 image_thereshold = otsuThreshold(original_image);

    for (i=1;i<MT9V03X_H-2;i++)
    {
        for (j =1 ; j<MT9V03X_W-2;j++)
        {
            float Difference_val;
            if (original_image[i][j]+original_image[i+1][j+1])
            {
                Difference_val=((original_image[i][j]-original_image[i+1][j+1])+0.0f)/(original_image[i][j]+original_image[i+1][j+1]);

                if (fabs(Difference_val)>0.05)
                {
                  if (original_image[i][j]>original_image[i+1][j+1] && original_image[i][j]>image_thereshold && original_image[i+1][j+1]<image_thereshold)
                      Difference_image[i][j]=white;
                  else if (original_image[i][j]<original_image[i+1][j+1] && original_image[i][j]<image_thereshold && original_image[i+1][j+1]>image_thereshold)
                      Difference_image[i][j]=white;
                  else
                      Difference_image[i][j]=black;
                }
                else
                {
                Difference_image[i][j]=black;
                }

            }
            else
            {
                Difference_image[i][j]=black;
            }

        }
    }

}



uint8 bin_Multi_threshold[MT9V03X_H][MT9V03X_W];//用来存放二值化之后的图像

//大津法
uint8 otsuThreshold(uint8 tmImage[MT9V03X_H][MT9V03X_W])
{
  int16 i,j;//循环计数器

  uint32 Amount = 0;//像素总数
  uint32 PixelIntegral = 0; //所有像素的灰度值总和

  uint32 PixelBack = 0; //前景像素点数
  uint32 PixelIntegralBack = 0;//前景灰度值总和

  int32 PixelFore = 0; //背景的像素点数
  int32 PixelIntegralFore = 0; //背景的灰度值总和
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
  //前景和背景像素所占的百分比      前景和背景的平均灰度     当前和最大的类间方差
  uint8 MinValue, MaxValue; //图像中的最小和最大灰度值
  uint8 Threshold = 0;//计算出的最佳阈值
  uint8 HistoGram[256];// 灰度直方图，用于统计每个灰度级在图像中出现的次数

  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图

  for (j = 0; j < MT9V03X_H; j++)
  {
    for (i = 0; i < MT9V03X_W; i++)
    {
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    }
  }

  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; MaxValue--) ; //获取最大灰度的值

//  printf("%d-%d\n",HistoGram[MinValue],HistoGram[MaxValue]);
//  printf("最大最小像素点%d-%d\n",MinValue,MaxValue);

  if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色
  if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色

  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数

  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];   //前景像素点数
    PixelFore = Amount - PixelBack;         //背景像素点数
    OmegaBack = (double)PixelBack / Amount;//前景像素百分比
    OmegaFore = (double)PixelFore / Amount;//背景像素百分比
    PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
    MicroBack = (double)PixelIntegralBack / PixelBack;   //前景平均灰度值
    MicroFore = (double)PixelIntegralFore / PixelFore;   //背景平均灰度值
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = (uint8)j;
    }
  }
  return Threshold;  //返回最佳阈值;
}


//二值化

void turn_to_bin()
{
    uint8 i, j;
    get_original();
    uint8 image_thereshold = otsuThreshold(original_image);
//  printf("单阈值%d\n",image_thereshold);
  for(i = 0;i<MT9V03X_H;i++)
  {
      for(j = 0;j<MT9V03X_W;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_Multi_threshold[i][j] = white;
          else bin_Multi_threshold[i][j] = black;
      }
  }
}





// 冒泡排序函数
void bubbleSort(uint8 arr[], uint8 n) {
    uint8 i, j, temp;
    for (i = 0; i < n-1; i++) {     // 外层循环，控制遍历次数
        for (j = 0; j < n-i-1; j++) { // 内层循环，控制比较次数
            if (arr[j] > arr[j+1]) { // 如果当前元素大于下一个元素
                // 交换两个元素
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}



#define SCALE_FACTOR 255.0 / log(256.0) // 缩放因子，确保结果值在0-255范围内

uint8 median_image[MT9V03X_H][MT9V03X_W];
// 二维中值滤波函数
void medianFilter2D() {

    uint8 filterSize=3;
    uint8 halfSize = filterSize / 2;  //用来忽略边界
    uint8 i,j;

    for(i=0;i<halfSize;i++)//填充边界
    {
        for (j=0;j<MT9V03X_W;j++)
        {
            median_image[i][j]=mt9v03x_image[i][j];
            median_image[j][i]=mt9v03x_image[j][i];
            median_image[MT9V03X_H-1-i][j]=mt9v03x_image[MT9V03X_H-1-i][j];
            median_image[MT9V03X_W-1-j][i]=mt9v03x_image[MT9V03X_W-1-j][i];
        }

    }


    for (i = halfSize; i < MT9V03X_H - halfSize; i++) {
        for (j = halfSize; j < MT9V03X_W - halfSize; j++) {
            uint8 window[filterSize * filterSize];
            uint8 idx = 0;


            // 填充窗口
            for (int8 m = -halfSize; m <= halfSize; m++) {
                for (int8 n = -halfSize; n <= halfSize; n++) {
                    window[idx++] = mt9v03x_image[i + m][j + n];
                }
            }

            // 对窗口进行排序
            bubbleSort(window,idx);
            // 取中值
            median_image[i][j] =window[idx/2] ;

        }
    }
}








void Multi_threshold(void)
{
  //medianFilter2D();
  uint8 i,j;
  get_original();
  uint8 image_thereshold = otsuThreshold(original_image);
  for(i = 0;i<MT9V03X_H;i++)
  {
      for(j = 0;j<MT9V03X_W;j++)
      {
          if(original_image[i][j]<image_thereshold) bin_Multi_threshold[i][j] = image_thereshold;
          else bin_Multi_threshold[i][j] = original_image[i][j];
      }

  }
  image_thereshold = otsuThreshold(bin_Multi_threshold);
//  printf("多阈值%d\n",image_thereshold);
  for(i = 0;i<MT9V03X_H;i++)
    {
        for(j = 0;j<MT9V03X_W;j++)
        {
            if(bin_Multi_threshold[i][j]<image_thereshold)bin_Multi_threshold[i][j] = black;
            else bin_Multi_threshold[i][j] = white;

        }
    }

}






void findPeak(uint8 v[], uint8 *peakPositions) {
    // 分配内存来存储差分数组
    int8 diff_v[255];

    uint16 i;
    // 计算V的一阶差分
    for (i = 0; i < 255; i++) {
        if (v[i + 1] - v[i] > 0)
            diff_v[i] = 1;
        else if (v[i + 1] - v[i] < 0)
            diff_v[i] = -1;
        else
            diff_v[i] = 0;
    }
    // 找出峰值位置
    for (i = 0; i <255; i++) {
        if (diff_v[i] == 1 && (i == 254 || diff_v[i + 1] == -1)) {
            // 如果当前是1，且下一个是-1或已经是最后一个元素

            if (v[i]>peakPositions[2])
            {
                peakPositions[0]=peakPositions[1];
                peakPositions[1]=peakPositions[2];
                peakPositions[2]=v[i];
            }

        }
    }
}



uint8 vague_image[MT9V03X_H][MT9V03X_W];

void vague()
{
   uint16 i,j;
   uint8 HistoGram[256];// 灰度直方图，用于统计每个灰度级在图像中出现的次数

   for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图

   for (j = 0; j < MT9V03X_H; j++)
   {
   for (i = 0; i < MT9V03X_W; i++)
   {
      HistoGram[mt9v03x_image[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
   }
   }

   uint8 peakPositions[3]={0,0,0};
   findPeak(HistoGram,peakPositions);
//   printf("峰值%d  %d  %d\n",peakPositions[0],peakPositions[1],peakPositions[2]);

    uint8 threshold=(peakPositions[1]+peakPositions[2])/2;

    for(i = 0;i<MT9V03X_H;i++)
      {
          for(j = 0;j<MT9V03X_W;j++)
          {
              if(original_image[i][j]>threshold)vague_image[i][j] = white;
              else vague_image[i][j] = black;
          }
      }


}





