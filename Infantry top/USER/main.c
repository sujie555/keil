#include "main.h"
int cou=0;
/**********************************************************
�°��ӵ����ű仯
ң����(δ��)PB7
CANͨ�� (�б�)PB8/CAN1_RX  PB9/CAN1_TX   �Ѹ�
���� PB0/TIM3  CH3
Ԥ��������IO��һ�����ڣ� ����PA2/USART2_TX PA3/USART2_RX  PA7/IO  PA6/RST
����3��6����  
ʱ��ͨ�� PA1/TIM5_CH2   PA0/TIM5_CH1   PB5/TIM3_CH2
����Ҫ��һ������¼��������ʱ��Ķ�ʱ�����ˣ���TIM5��ΪTIM2
**********************************************************/
/**********************************************************
���ڿ��Ź�
��һ��7λ�ĵݼ�����������ι����ʱ���������ֵ�����趨ֵʱ������
����������ֵ��0x40��Ϊ0x3Fʱ�����Ź����Ḵλ
���ʹ�ܿ��Ź����������жϣ�����������ֵ��Ϊ0x40ʱ�򣬾ͻ����һ���ж�
����ι��������ϵͳ��λ
**********************************************************/
/**********************************************************
���������⼰�����
���ӷ�Ӧ������б�º���ȥ��
Ħ�����ٶȣ��ĳ����֣��������ʱ���٣���������¸���
����ʱǹͷб�ţ�����P������ʽ�ĳ�ŷ���ǿ���
���ʼ���������ȡ����ϵͳ���ݣ����ɹ��ʱջ�
ģʽ���ӣ�ʵ���˵��̺���̨���룬���̿�����Ť��̨����
�����Period����20000�����һ��
M R FF FF FF FF FF 0A

��סr���������ջ�ȡ��
**********************************************************/
int i=0;
int main()
{
  delay_ms(2000);
	engineerpower_Init();
	BSP_Init();

	ControtLoopTaskInit();

	RemoteTaskInit();
	delay_ms(1000);

	TIM6_Start();
	//WWDG_Init();

	delay_ms(100);

	while(1)
	{
	
	  delay_ms(1);
		
//    CM1SpeedPID.fdb = CM1Encoder.filter_rate;	  
//		CM1SpeedPID.kp = 30;
//		CM1SpeedPID.ki = 5;
//		CM1SpeedPID.kd = 5;
//		CM1SpeedPID.Calc(&CM1SpeedPID);

//Set_CM_Speed(CAN1, 50,0,0,0);
//		printf("%f\t",chassisPowerError);
	//printf("%d\r\n",ChassisSpeedRef.forward_back_ref);
//		printf("%f\t",GMYPositionPID.ref);
//		printf("%f\t",GMYPositionPID.fdb);
//		printf("%f\r\n",CMRotatePID.fdb);
//		now_system_micrsecond = Get_Time_Micros();	
//		delay_ms(500);
		cou++;

	}
}
