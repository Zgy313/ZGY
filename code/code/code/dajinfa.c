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



uint8 bin_Multi_threshold[MT9V03X_H][MT9V03X_W];//������Ŷ�ֵ��֮���ͼ��

//���
uint8 otsuThreshold(uint8 tmImage[MT9V03X_H][MT9V03X_W])
{
  int16 i,j;//ѭ��������

  uint32 Amount = 0;//��������
  uint32 PixelIntegral = 0; //�������صĻҶ�ֵ�ܺ�

  uint32 PixelBack = 0; //ǰ�����ص���
  uint32 PixelIntegralBack = 0;//ǰ���Ҷ�ֵ�ܺ�

  int32 PixelFore = 0; //���������ص���
  int32 PixelIntegralFore = 0; //�����ĻҶ�ֵ�ܺ�
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
  //ǰ���ͱ���������ռ�İٷֱ�      ǰ���ͱ�����ƽ���Ҷ�     ��ǰ��������䷽��
  uint8 MinValue, MaxValue; //ͼ���е���С�����Ҷ�ֵ
  uint8 Threshold = 0;//������������ֵ
  uint8 HistoGram[256];// �Ҷ�ֱ��ͼ������ͳ��ÿ���Ҷȼ���ͼ���г��ֵĴ���

  for (j = 0; j < 256; j++)  HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

  for (j = 0; j < MT9V03X_H; j++)
  {
    for (i = 0; i < MT9V03X_W; i++)
    {
      HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    }
  }

  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ

//  printf("%d-%d\n",HistoGram[MinValue],HistoGram[MaxValue]);
//  printf("�����С���ص�%d-%d\n",MinValue,MaxValue);

  if (MaxValue == MinValue)     return MaxValue;         // ͼ����ֻ��һ����ɫ
  if (MinValue + 1 == MaxValue)  return MinValue;        // ͼ����ֻ�ж�����ɫ

  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ��������

  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;//�Ҷ�ֵ����
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];   //ǰ�����ص���
    PixelFore = Amount - PixelBack;         //�������ص���
    OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
    OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
    PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
    MicroBack = (double)PixelIntegralBack / PixelBack;   //ǰ��ƽ���Ҷ�ֵ
    MicroFore = (double)PixelIntegralFore / PixelFore;   //����ƽ���Ҷ�ֵ
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//������䷽��
    if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
    {
      SigmaB = Sigma;
      Threshold = (uint8)j;
    }
  }
  return Threshold;  //���������ֵ;
}


//��ֵ��

void turn_to_bin()
{
    uint8 i, j;
    get_original();
    uint8 image_thereshold = otsuThreshold(original_image);
//  printf("����ֵ%d\n",image_thereshold);
  for(i = 0;i<MT9V03X_H;i++)
  {
      for(j = 0;j<MT9V03X_W;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_Multi_threshold[i][j] = white;
          else bin_Multi_threshold[i][j] = black;
      }
  }
}





// ð��������
void bubbleSort(uint8 arr[], uint8 n) {
    uint8 i, j, temp;
    for (i = 0; i < n-1; i++) {     // ���ѭ�������Ʊ�������
        for (j = 0; j < n-i-1; j++) { // �ڲ�ѭ�������ƱȽϴ���
            if (arr[j] > arr[j+1]) { // �����ǰԪ�ش�����һ��Ԫ��
                // ��������Ԫ��
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}



#define SCALE_FACTOR 255.0 / log(256.0) // �������ӣ�ȷ�����ֵ��0-255��Χ��

uint8 median_image[MT9V03X_H][MT9V03X_W];
// ��ά��ֵ�˲�����
void medianFilter2D() {

    uint8 filterSize=3;
    uint8 halfSize = filterSize / 2;  //�������Ա߽�
    uint8 i,j;

    for(i=0;i<halfSize;i++)//���߽�
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


            // ��䴰��
            for (int8 m = -halfSize; m <= halfSize; m++) {
                for (int8 n = -halfSize; n <= halfSize; n++) {
                    window[idx++] = mt9v03x_image[i + m][j + n];
                }
            }

            // �Դ��ڽ�������
            bubbleSort(window,idx);
            // ȡ��ֵ
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
//  printf("����ֵ%d\n",image_thereshold);
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
    // �����ڴ����洢�������
    int8 diff_v[255];

    uint16 i;
    // ����V��һ�ײ��
    for (i = 0; i < 255; i++) {
        if (v[i + 1] - v[i] > 0)
            diff_v[i] = 1;
        else if (v[i + 1] - v[i] < 0)
            diff_v[i] = -1;
        else
            diff_v[i] = 0;
    }
    // �ҳ���ֵλ��
    for (i = 0; i <255; i++) {
        if (diff_v[i] == 1 && (i == 254 || diff_v[i + 1] == -1)) {
            // �����ǰ��1������һ����-1���Ѿ������һ��Ԫ��

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
   uint8 HistoGram[256];// �Ҷ�ֱ��ͼ������ͳ��ÿ���Ҷȼ���ͼ���г��ֵĴ���

   for (j = 0; j < 256; j++)  HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

   for (j = 0; j < MT9V03X_H; j++)
   {
   for (i = 0; i < MT9V03X_W; i++)
   {
      HistoGram[mt9v03x_image[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
   }
   }

   uint8 peakPositions[3]={0,0,0};
   findPeak(HistoGram,peakPositions);
//   printf("��ֵ%d  %d  %d\n",peakPositions[0],peakPositions[1],peakPositions[2]);

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





