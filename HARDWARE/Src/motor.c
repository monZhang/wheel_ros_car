#include "motor.h"


/**************************************************************************
�������ܣ�ʹ�ܿ������ų�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void Enable_Pin(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //KEY��Ӧ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB1
}

void TIM1_PWM_Init(u16 arr, u16 psc) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);      //TIM8ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);    //ʹ��PORTCʱ��

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;   //GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  //�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);       //��ʼ��PC��

    //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Period = arr;
    //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_ClockDivision = 1;
    //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    //�߼���ʱ���������ʹ�����
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    //CH1Ԥװ��ʹ��
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    //ʹ��TIM8
    TIM_Cmd(TIM1, ENABLE);
}

void TIM9_PWM_Init(u16 arr, u16 psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);/*ʹ�ܶ�ʱ��11ʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);/*ʹ��GPIOFʱ��*/

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);/*����*/
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);/*����*/

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;           //GPIOF9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);              //��ʼ��PF9

    TIM_TimeBaseInitStructure.TIM_Period = arr;/*�Զ���װ��*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*Ԥ��Ƶ*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*ʱ�ӷ�Ƶ*/
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*���ϼ���*/
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseInitStructure);/*��ʼ��*/

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWMģʽ*/
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*���*/
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*�Ƚϼ��Ը�*/
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);/*����Ƚ�Ԥװ��ʹ��*/
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);/*����Ƚ�Ԥװ��ʹ��*/

    TIM_ARRPreloadConfig(TIM9, ENABLE);/*�Զ�����Ԥװ��ʹ��*/

    TIM_Cmd(TIM9, ENABLE);/*����ʹ��*/

}


void TIM10_PWM_Init(u16 arr, u16 psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/*ʹ��GPIOFʱ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);/*ʹ�ܶ�ʱ��11ʱ��*/

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);/*����*/

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*����*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*�������*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;/*PF7*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*����*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
    GPIO_Init(GPIOB, &GPIO_InitStructure);/*��ʼ��IO*/

    TIM_TimeBaseInitStructure.TIM_Period = arr;/*�Զ���װ��*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*Ԥ��Ƶ*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*ʱ�ӷ�Ƶ*/
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*���ϼ���*/
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseInitStructure);/*��ʼ��*/

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWMģʽ*/
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*���*/
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*�Ƚϼ��Ը�*/
    TIM_OC1Init(TIM10, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);/*����Ƚ�Ԥװ��ʹ��*/

//TIM_CtrlPWMOutputs(TIM10,ENABLE);

    TIM_ARRPreloadConfig(TIM10, ENABLE);/*�Զ�����Ԥװ��ʹ��*/

    TIM_Cmd(TIM10, ENABLE);/*����ʹ��*/

}

void TIM11_PWM_Init(u16 arr, u16 psc) {

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/*ʹ��GPIOFʱ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);/*ʹ�ܶ�ʱ��11ʱ��*/

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);/*����*/

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/*����*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*�������*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;/*PF7*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*����*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
    GPIO_Init(GPIOB, &GPIO_InitStructure);/*��ʼ��IO*/

    TIM_TimeBaseInitStructure.TIM_Period = arr;/*�Զ���װ��*/
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*Ԥ��Ƶ*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*ʱ�ӷ�Ƶ*/
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*���ϼ���*/
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseInitStructure);/*��ʼ��*/

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWMģʽ*/
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*���*/
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*�Ƚϼ��Ը�*/
    TIM_OC1Init(TIM11, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);/*����Ƚ�Ԥװ��ʹ��*/

    TIM_CtrlPWMOutputs(TIM11, ENABLE);

    TIM_ARRPreloadConfig(TIM11, ENABLE);/*�Զ�����Ԥװ��ʹ��*/

    TIM_Cmd(TIM11, ENABLE);/*����ʹ��*/

}








