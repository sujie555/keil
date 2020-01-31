#ifndef __GUN_H__
#define __GUN_H__
#include <stm32f4xx.h>

/***************************************
ʱ��ͨ�� PA1/TIM5_CH2(��Ħ����)   
PA0/TIM5_CH1(��Ħ����)   
PB5/TIM3_CH2(���)
PB0/TIM3 CH3(����)
***************************************/
#define PWM1_PA0  TIM5->CCR1//��Ħ����
#define PWM1_PA1  TIM5->CCR2//��Ħ����
#define PB5_HELM_MOTOR  TIM3->CCR2//���
#define PB0_LASER_L TIM3->CCR3//����


//1ms 0%����, 2ms100%����,��Ӧ1000~2000
#define InitFrictionWheel()     \
        PWM1_PA0 = 1000;             \
        PWM1_PA1 = 1000;
#define SetFrictionWheelSpeed1(x) \
        PWM1_PA0 = x;                
     
#define SetFrictionWheelSpeed2(x) \
        PWM1_PA1 = x;				
				

#define SET_HELM    PB5_HELM_MOTOR
#define SET_LASER	  PBout(0)//PB0_LASER_L
typedef struct PWM_Set
{
	uint32_t RCC_AHB1Periph;
	uint32_t GPIO_Pin_x;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_PinSource;
  uint8_t GPIO_AF;
	uint32_t Prescaler;
	uint32_t Period;
	TIM_TypeDef* TIMx;
	uint32_t TIM_Pulse;
	uint32_t Chx;
}PWM_Set;


void PWM_Configuration(void);
void PWM_Configuration_AUTO(PWM_Set* PWM);
void Switch_GPIO_PinSource(PWM_Set* PWM);

#endif
