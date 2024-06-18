#include <Encoder.h>
#include <zf_driver_timer.h>

SPEED speed1;
SPEED speed2;


//���������ٳ�ʼ��
void SpeedCal_Init(SPEED *speed,gpio_pin_enum A,gpio_pin_enum B)
{
    //������ʼ��
    speed->DIR=0;
    speed->ENCODER.Kg=0.02f;
    speed->ENCODER.LastP=0;
    speed->ENCODER.Now_P=0;
    //��������Э����,Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
    //��������Э����,R���󣬶�̬��Ӧ�����������ȶ��Ա��
    speed->ENCODER.Q=0.0118f;
    speed->ENCODER.R=0.6480f;
    speed->ENCODER.out=0;
    speed->Flag=0;
    speed->PinA=A;
    speed->PinB=B;
    speed->PinAState=0;
    speed->PinBState=0;
    speed->T=0;
    speed->lastspeed=0;
    speed->speed=0;
    //1�ŵ������������
    //GPIO �ⲿ�жϳ�ʼ��  P15_5  P15_3
    gpio_init(speed->PinA,GPI,0,GPI_PULL_DOWN); //A��
    gpio_init(speed->PinB,GPI,0,GPI_PULL_DOWN); //B��
    exti_init(ERU_CH4_REQ13_P15_5,EXTI_TRIGGER_RISING);
    //2�ŵ������������ p15_4  p15_2
    exti_init(ERU_CH0_REQ0_P15_4,EXTI_TRIGGER_RISING);
    system_start();         //��ʼ��ʱ


}

//�ٶȼ���
void SpeedCal(SPEED *speed)
{
    speed->Flag++;
    switch(speed->Flag)
    {
        case 1:

            speed->PinAState=gpio_get_level(speed->PinA);
            speed->PinBState=gpio_get_level(speed->PinB);
            speed->DIR=(speed->PinAState==speed->PinBState)?(1):(-1);
            speed->M1T=system_getval_ms()/1000.0;

            break;
        case 2:
            speed->T=system_getval_ms()/1000.0-speed->M1T;
            // ����ֱ�� D=10cm T�����٣�V=(3.1415926*D)/(Z*T)   TΪ��������ļ������λΪs��,V�ĵ�λΪcm/s
            speed->speed=(Pi*D)/(Z*speed->T);
            speed->speed*=speed->DIR;
            if(speed->speed>speed->lastspeed+20)
            {
                speed->speed=speed->lastspeed+20;
            }else if(speed->speed<speed->lastspeed-20)
            {
                speed->speed=speed->lastspeed-20;
            }
            speed->speed=kalmanFilter(&speed->ENCODER,speed->speed);
            speed->lastspeed=speed->speed;
            speed->Flag=0;
            speed->M1T=0;
            speed->M2T=0;
            speed->T=0;
            speed->timeout=0;
            system_start();//����������ʱ��
            break;
        default :
            speed->Flag=0;
            speed->M1T=0;
            speed->M2T=0;
            speed->T=0;
            system_start();//����������ʱ��
            speed->speed=0;
            speed->timeout=0;
            break;

    }

}

//���ٳ�ʱ���(�ŵ�2ms��ʱ�ж���)
void Time_Out_Detect(SPEED *speed)
{

    if(speed->Flag==1)
    {
    speed->timeout++;
    }else if(speed->Flag==0)
    {
        speed->timeout=0;

    }
    if(speed->timeout>=5)//100ms
    {

        speed->Flag=0;
        speed->M1T=0;
        speed->M2T=0;
        speed->T=0;
        system_start();//����������ʱ��
        speed->speed=0;
        speed->timeout=0;

    }

}
