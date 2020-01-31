#include "main.h"
WorkState_e lastWorkState = STOP_STATE;
WorkState_e workState = STOP_STATE;
uint8_t camera_temp = 0;
uint8_t dafu_temp = 0;
uint8_t cat_temp = 0;    
//int identify_flag =0 ;      //���������Ƿ�Ϊս��ʶ��ģʽ
/**
������WorkStateFSM()
���ܣ����ƹ���ģʽ
**/
/////////////////////��������flag������
int flag_prepare_ready=0;
int flag_Dafu_fromsight=1;//0;//���Ӿ�������յ��˴������
////////////////////
void WorkStateFSM(void)
{
	lastWorkState = workState;//���µ�ǰ����״̬
	switch(workState)
	{
		case STOP_STATE:                            
		{
			if(GetInputMode() != STOP)//ң����ǿ��ֹͣ
			{
				workState = PREPARE_STATE;   //ȡ����ͣҪ�ȹ����
			}
		}break;
		
		case PREPARE_STATE://׼��״̬
		{
			if(GetInputMode() == STOP)//ң����ǿ��ֹͣ
			{
				workState = STOP_STATE;
			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)//׼��״̬�ﵽָ��ʱ���ת�������ֿ���״̬
			{
				workState = NORMAL_STATE;
			}
		}break;
		case NORMAL_STATE:     //�����ֿ���״̬�������룩
		{
			if(GetInputMode() == STOP)//ң����ǿ��ֹͣ
			{
				workState = STOP_STATE;
			}
//			else if(camera_temp == 1)//����c�����������ģʽ
//			{
////       S1inputmode = ZIMIAO;
//			}
			else if(dafu_temp == 1&&flag_Dafu_fromsight==1)//����f����ת��������״̬    (���) 
			{
				workState = CAMERA_STATE;//STANDBY_STATE;      
			}	
			else if(cat_temp == 1)//ң���������������룬ת��������״̬     
			{
				workState = CATWALK_STATE;///CAMERA_STATE;//STANDBY_STATE;      
			}	
		}break;
		
		case CAMERA_STATE:      //���״̬
		{
			if(GetInputMode() == STOP)//ң����ǿ��ֹͣ
			{
				workState = STOP_STATE;
			}	
			else if(dafu_temp == 0)//�ɿ�C����������̲���״̬
			{
				workState = NORMAL_STATE;      
			}
		}break;
	
		case ADJUST_STATE:      //��׼״̬
		{
//			if(GetInputMode() == STOP)
//			{
//				workState = STOP_STATE;
//			}
//			else if(adjust_temp == 0)
//			{
//				workState = NORMAL_STATE;      
//			}
		}break;
		
		case CATWALK_STATE:      
		{
			if(GetInputMode() == STOP)
			{
				workState = STOP_STATE;
			}
			else if(cat_temp == 0)
			{
				workState = NORMAL_STATE;      
			}
		}break;
		default:
		{
		}
	}	
}

/**
������WorkStateSwitchProcess()
���ܣ����������ģʽ�л���prapareģʽ��Ҫ��һϵ�в�����ʼ��
**/
void WorkStateSwitchProcess(void)
{
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		ControtLoopTaskInit();
		RemoteTaskInit();
		ChassisMotorInit();
		//AllDataInit();
	}
}
/**
������SetWorkState(WorkState_e state)
���ܣ����ص�ǰ����״̬

**/
void SetWorkState(WorkState_e state)
{
	if(workState != PREPARE_STATE)
	{
    workState = state;
	}
}

/**
������WorkState_e GetWorkState(void)
���ܣ����ù���״̬

**/
WorkState_e GetWorkState(void)
{
	return workState;
}

WorkState_e GetlastWorkState(void)
{
	return lastWorkState;
}

void AllDataInit(void)
{
	Key_State cameraKey = KEY_INIT;//zimiao���
	Key_State dafuKey = KEY_INIT;//�����
	Key_State servoKey = KEY_INIT;//�����
	Key_State frictionKey = KEY_INIT;//Ħ�����ٶ��л���
	uint8_t mouseLeftPress = 0;
	uint8_t continuous_shoot_Flag = 0;
	uint8_t quickShootFlag = 0;
	uint8_t servoPositionFlag = 0;
	uint8_t sendCustomDataFlag = 0;
	uint8_t camera_temp = 0;
	uint8_t dafu_temp = 0;
	uint8_t cat_temp = 0;
	Key_State xtl = KEY_INIT;//XTL
}
