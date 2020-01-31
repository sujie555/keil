#include "main.h"

ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;
RC_Ctl_t RC_CtrlData;
RC_GameInformation_t GameInformationData;
uint16_t Speed_Friction1 = 600;//375;//Ħ�����ٶ�,1000ֹͣ,2000���   �ұ�
uint16_t Speed_Friction2 = 600;//375;//Ħ�����ٶ�,1000ֹͣ,2000���   ���

RampGen_t frictionRamp = RAMP_GEN_DAFAULT;   //Ħ����б��
RampGen_t MouseXSpeedRamp = RAMP_GEN_DAFAULT; 
RampGen_t MouseYSpeedRamp = RAMP_GEN_DAFAULT; 
RampGen_t catGimbalCount = RAMP_GEN_DAFAULT; 
RampGen_t LRSpeedRamp  = RAMP_GEN_DAFAULT;   //mouse�����ƶ�б��
RampGen_t FBSpeedRamp  = RAMP_GEN_DAFAULT;   //mouseǰ���ƶ�б��
RampGen_t RoSpeedRamp  = RAMP_GEN_DAFAULT;	 //q e������ת

uint8_t IsRemoteBeingAction(void)
{
	return 	(fabs(GimbalRef.yaw_speed_ref)>=10 || \
		       fabs(ChassisSpeedRef.forward_back_ref)>=10 || \
	         fabs(ChassisSpeedRef.left_right_ref)>=10 || \
	         fabs(ChassisSpeedRef.rotate_ref)>=10);
}

/**
������RemoteDataPrcess(uint8_t *pData)
���ܣ���ң�����źŽ��д���
**/
void RemoteDataPrcess(uint8_t *pData)
{
	remote_micrsecond.time_last =Get_Time_Micros();
	if(pData == NULL)
	{
			return;
	}
	//ch0~ch3:max=1684,min=364,|error|=660
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; //ң����ͨ��0����������ƽ��
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;//ң����ͨ��1������ǰ������
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;//ң����ͨ��2��������ת
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;//ң����ͨ��3�����Ƹ���
	
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;//ң������߿��أ���3����λ��ң��������ģʽ�����ã���� RemoteShootControl
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//ң�����ұ߿��أ���3����λ������Ϊǿ��ֹͣ������Ϊң�������ƣ��м�Ϊ���̿���

	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);//�������
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);//�������
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    //û�õ�

	RC_CtrlData.mouse.press_l = pData[12];//������
	RC_CtrlData.mouse.press_r = pData[13];//����Ҽ�

	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);//ÿһλ��Ӧһ������
	
/**
0x0001:w 
0x0002:s 
0x0004:a 
0x0008:d 

0x0010:shift 
0x0020:ctrl 
0x0040:q 
0x0080:e 

0x0100:r 
0x0200:f 
0x0400:g 
0x0800:z 

0x1000:x 
0x2000:c 
0x4000:v 
0x8000:b
**/
		
		SetInputMode(&RC_CtrlData.rc);//���ң�����ұ߿��أ�ѡ��ģʽ
		SetS1InputMode(&RC_CtrlData.rc);//�����߿���״̬��ѡ������

		switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			//ң��������ģʽ
			RemoteControlProcess(&(RC_CtrlData.rc));
		}break;
		case KEY_MOUSE_INPUT:
		{
			//�������ģʽ
			MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
		}break;
		case STOP:
		{
			//����ͣ��
			InitFrictionWheel();
			frictionwheelmotor( 0 , 0 );
			Set_CM_Speed(CAN1, 0,0,0,0);
			Set_Gimbal_Current(CAN1, 0, 0, 0);
		}break;
	}
	

//	wwdg_flag |=0x0040;
}


/*****************
����:JudgementHandle(uint8_t *pData)
���ܣ�����ϵͳ���ݴ���
*****************/
void JudgementHandle(uint8_t *pData)
{
  GameInformationData.CommandCodeID = (int16_t)pData[5];
	GameInformationData.TimeRemain = ((int32_t)pData[7]<<24)|((int32_t)pData[8]<<16)|((int32_t)pData[9]<<8)|((int32_t)pData[10]);
	GameInformationData.BloodRemain = ((int16_t)pData[12]<<8)|((int16_t)pData[11]);                                                                                                                                                                           
	GameInformationData.Voltage = ((int32_t)pData[16]<<24)|((int32_t)pData[15]<<16)|((int32_t)pData[14]<<8)|((int32_t)pData[13]);
	GameInformationData.Current = ((int32_t)pData[20]<<24)|((int32_t)pData[18]<<19)|((int32_t)pData[18]<<8)|((int32_t)pData[17]);
	GameInformationData.RemainPower = ((int32_t)pData[41]<<24)|((int32_t)pData[40]<<16)|((int32_t)pData[39]<<8)|((int32_t)pData[38]);
	GameInformationData.ecd_Voltage = Parameter_Transformation(GameInformationData.Voltage);
	GameInformationData.ecd_Current = Parameter_Transformation(GameInformationData.Current);
	GameInformationData.ChassisPower = GameInformationData.ecd_Voltage*GameInformationData.ecd_Current;
}
//ң�������ݳ�ʼ����б�º����ȵĳ�ʼ��
void RemoteTaskInit(void)
{
	//б�³�ʼ��
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	MouseXSpeedRamp.SetScale (&MouseXSpeedRamp , MOUSR_MOUSEX_RAMP_TICK_COUNT);
	MouseYSpeedRamp.SetScale (&MouseYSpeedRamp , MOUSR_MOUSEY_RAMP_TICK_COUNT);
	catGimbalCount.SetScale(&catGimbalCount, 150);
	LRSpeedRamp.SetScale (&LRSpeedRamp , 20);
	FBSpeedRamp.SetScale (&FBSpeedRamp , 30);
	RoSpeedRamp.SetScale (&RoSpeedRamp , MOUSE_Ro_RAMP_TICK_COUNT);
	
	frictionRamp.ResetCounter(&frictionRamp);
	MouseXSpeedRamp.ResetCounter(&MouseXSpeedRamp);
	MouseYSpeedRamp.ResetCounter(&MouseYSpeedRamp);
	catGimbalCount.ResetCounter(&catGimbalCount);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	RoSpeedRamp.ResetCounter(&RoSpeedRamp);
	
	//������̨����ֵ��ʼ��
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	//Ħ��������״̬��ʼ��
	SetFrictionState(FRICTION_WHEEL_OFF);
	InitFrictionWheel();
}
