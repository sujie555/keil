#include "main.h"

uint16_t wwdg_flag = 0xffff;
 
/**********************************
* FUNCTION NAME��WWDG_Init
* FUNCTION �����Ź����ܳ�ʼ��
* MODIFY DATE ��2017
* INPUT ����
* RETURN ����
***********************************/
void WWDG_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE); //ʹ�ܴ��ڿ��Ź�ʱ��
	
	/* -------------- Enable The WWDG ---------------------------------------*/
	
		WWDG_SetPrescaler(WWDG_Prescaler_8); //���÷�Ƶֵ
		WWDG_SetWindowValue(0x42); //���ô���ֵ  0100 0010 
		//0x40���´��ڵ�ֵ,�ϴ��ڵ�ֵ�Լ���
		WWDG_Enable(0x43);  //�������Ź�			   0100 0011
		WWDG_ClearFlag();//�����ǰ�����жϱ�־λ
	
}

/**********************************
* FUNCTION NAME��ShootSupervise
* FUNCTION ������������ӳ��򣬷�ֹ����
* MODIFY DATE ��2018.3.14
* INPUT ����
* RETURN ����
***********************************/
void ShootSupervise(void)
{
	float temp;
		temp = (CM7Encoder.ecd_angle/51.4285714 +0.1+ count_temp);
		if(temp>-2)  
	{
	}
		else
	{
		count_temp = (int16_t)(CM7Encoder.ecd_angle/51.4285714 -2.1);
	}
}

/**********************************
* FUNCTION NAME��SuperiviseTask
* FUNCTION ������������ӳ���
* MODIFY DATE ��2017
* INPUT ����
* RETURN ��0 1��Ŀǰ�����ΪʲôҪ��ôд��
***********************************/
int SuperiviseTask(void)
{
	/* -------------- ���ң������mpu6050 -----------------------------------*/
	{
		mpu6050_micrsecond.time_now   = Get_Time_Micros();
		mpu6050_micrsecond.time_error = mpu6050_micrsecond.time_now - mpu6050_micrsecond.time_last;
		remote_micrsecond.time_now    = Get_Time_Micros();
		remote_micrsecond.time_error  = remote_micrsecond.time_now - remote_micrsecond.time_last;
		wwdg_flag &= ~0x0180;
		if(remote_micrsecond.time_error > 14500)//ң�����ֲ���д�����ݴ�������Ϊ7ms,����ʵ��Ϊ14ms
		{
			wwdg_flag |= 0x0080;
		}
		if(remote_micrsecond.time_error > 6500)//mpu6050���ݴ����ٶ�,ʵ��Ϊ4000~6000ms(�ܱ���ж�Ӱ��)
		{
			wwdg_flag |= 0x0100;
		}
	}
	/* -------------- ��������,��ι�� ---------------------------------------*/
	{
		if((wwdg_flag & 0x01ff) ==0x0000)      
		{
		 WWDG_SetCounter(0x43);             //ι��
			wwdg_flag = 0;
			return 1;
		}
		else
		{
			return 0;
		}
	}
}
