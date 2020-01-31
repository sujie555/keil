#include "main.h"
#include "math.h"
static RemoteSwitch_t switch1;   //ң������ದ��
Key_State_R dafuKey_R = KEY_INIT;//�����
void GetRemoteSwitchAction(RemoteSwitch_t *switch1, uint8_t s1)
{
	static uint32_t switch_cnt = 0;

	/* ����״ֵ̬ */
	switch1->switch_value_raw = s1;
	switch1->switch_value_buf[switch1->buf_index] = switch1->switch_value_raw;

	/* ȡ����ֵ����һ��ֵ */
	switch1->switch_value1 = (switch1->switch_value_buf[switch1->buf_last_index] << 2)| (switch1->switch_value_buf[switch1->buf_index]);


	/* ���ϵ�״ֵ̬������ */
	switch1->buf_end_index = (switch1->buf_index + 1) % REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* �ϲ�����ֵ */
	switch1->switch_value2 = (switch1->switch_value_buf[switch1->buf_end_index]<<4) | switch1->switch_value1;	

	/* �����ж� */
	if(switch1->switch_value_buf[switch1->buf_index] == switch1->switch_value_buf[switch1->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
	{
		switch1->switch_long_value = switch1->switch_value_buf[switch1->buf_index]; 	
	}

	//����ѭ��
	switch1->buf_last_index = switch1->buf_index;
	switch1->buf_index++;		
	if(switch1->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		switch1->buf_index = 0;	
	}			
}
/**
������RemoteShootControl(RemoteSwitch_t *switch1, uint8_t s1) 
���ܣ����ң������߿��ص�״̬
������̣�
���شӱ��λ���ƶ��������棬��һ�δ�Ħ���ۣ��ڶ��ιر�Ħ����
�����м�λ��Ϊ����״̬
��Ħ���۴�ʱ���ƶ���������Ļ�����򿪲��֣�������
**/
void RemoteShootControl(RemoteSwitch_t *switch1, uint8_t s1) 
{
	GetRemoteSwitchAction(switch1, s1);//���°���״̬������õ���һ�κ���һ�ε�״̬
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{	
			SetShootState(NOSHOOTING);
			frictionwheelmotor(0,0);
			if(switch1->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�ӹرյ�start turning
			{
				friction_wheel_state = FRICTION_WHEEL_ON;	 
			}
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(switch1->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   //�������ͱ��ر�
			{
				SetShootState(NOSHOOTING);
				frictionwheelmotor(0,0);
				friction_wheel_state = FRICTION_WHEEL_OFF;
			}
			else
			{
				//Ħ���ּ���
				//frictionwheelmotor(80*frictionRamp.Calc(&frictionRamp),80*frictionRamp.Calc(&frictionRamp));
				TIM3->CCR3 = 4000;
				//if(frictionRamp.IsOverflow(&frictionRamp))
				//{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
				//}
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(switch1->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)//REMOTE_SWITCH_CHANGE_1TO3��   //�ر�Ħ����
			{
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				frictionwheelmotor(0,0);
				//TIM3->CCR3 = 0;
				SetShootState(NOSHOOTING);
			}
			else 
			   {
					 
					 frictionwheelmotor(-50,-50);
					 ChassisSpeedRef.rotate_ref=50;
					 CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		       CM2SpeedPID.ref =  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
					 CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
					 CM4SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
		
			       if(switch1->switch_value_raw == 2)   //���ֿ�ʼת
			       {
				
				        SetShootState(SHOOTING);
							  shot_frequency_limt.time_now = Get_Time_Micros();//��ȡ��Ƭ������ʱ��     //TIM2�ļ�����ֵ��32λ���������
							  shot_frequency_limt.time_error = shot_frequency_limt.time_now - shot_frequency_limt.time_last;
							if(shot_frequency_limt.time_error > 1500000)
								{
					       count_temp ++;
					       shot_frequency_limt.time_last = Get_Time_Micros();  
				        }
			       }
//			else
//			{
//				SetShootState(NOSHOOTING);
//			}
		} break;
		default:break;
	}
/*			if(switch1->switch_value_raw == 2&&GetInputMode()==REMOTE_INPUT)
			{
				frictionwheelmotor(-250,-250);
				SetShootState(SHOOTING);
				shot_frequency_limt.time_now = Get_Time_Micros();     //TIM2�ļ�����ֵ��32λ���������
				shot_frequency_limt.time_error = shot_frequency_limt.time_now - shot_frequency_limt.time_last;
				if(shot_frequency_limt.time_error > 100000)
				{
					count_temp ++;
					shot_frequency_limt.time_last = Get_Time_Micros();  
				}
			}
			*/
////		 if(switch1->switch_value_raw == 2&&GetInputMode()==REMOTE_INPUT)   //����������ģʽ
////			{
////				 dafuKey_R.thisState =1;
////			}
////			else
////			{
////			   dafuKey_R.thisState =0;
////			}
	//		else 
//				if(GetInputMode()==REMOTE_INPUT)
//			{
//				SetShootState(NOSHOOTING);
//				frictionwheelmotor(0,0);
//			}
			DafuKeyCtrl_R();
			dafuKey_R.lastState = dafuKey_R.thisState;
    }
}

void DafuKeyCtrl_R(void)
 {
	if((dafuKey_R.thisState ==0)&&(dafuKey_R.lastState==1))
	{
		dafuKey_R.keyCnt ++;
		dafuKey_R.keyFlag = 1;
	}
	if(dafuKey_R.keyCnt%2==1)
	{		
		dafu_temp =1;
	}
	else
	{
		dafu_temp =0;
	}
}
/**
������RemoteControlProcess(Remote *rc)
���ܣ�ң�������ݸ���
**/
void RemoteControlProcess(Remote *rc)
{
	if(GetWorkState()!=PREPARE_STATE && GetWorkState()!=CAMERA_STATE)//�ֱ�Ϊǰ�����ҡ���������ת
	{
       if(friction_wheel_state == FRICTION_WHEEL_ON)
			 {
           ChassisSpeedRef.forward_back_ref   = -(rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*cos(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT
				                                        -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*sin(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
		       ChassisSpeedRef.left_right_ref     = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*sin(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT
				                                        -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*cos(GMYawEncoder.ecd_angle/180*PI) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
			 }
			 else
			 {
		   ChassisSpeedRef.forward_back_ref   = -(rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;
//		if(ChassisSpeedRef.forward_back_ref>100)ChassisSpeedRef.forward_back_ref=100;
//		if(ChassisSpeedRef.forward_back_ref< -100)ChassisSpeedRef.forward_back_ref= -100;
				//�ĳ���λ����VAL_LIMIT
		   ChassisSpeedRef.left_right_ref   = -(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_LR_REF_FACT; 
       ChassisSpeedRef.rotate_ref       =  (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;	
			 }
			 GimbalRef.pitch_angle_dynamic_ref += -(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;   
			 GimbalRef.yaw_angle_dynamic_ref   += -(rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT*2;
	  //�󳵰�����������ChassisSpeedRef.rotate_ref = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_FB_REF_FACT;//Ϊ�˰��������ǣ�ȡ����������
	}
	else
	{}
		/*  ��λ  */
		VAL_LIMIT(ChassisSpeedRef.forward_back_ref,-300,300);
		VAL_LIMIT(ChassisSpeedRef.left_right_ref,-170,170);//��֪��Ҫ��Ҫ
		VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref, -30, 10);
	// VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref,  -30, 30);
	 //ң�����������ݴ���	
		RemoteShootControl(&switch1, rc->s1);
	    ShooterMControlLoop();
//			SetGimbalMotorOutput();
}
