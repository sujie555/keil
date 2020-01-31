#include "main.h"
#include "KALMANTEST.h"

#define YAW_POSITION_KP_DEFAULTS  25
#define YAW_POSITION_KI_DEFAULTS  0
#define YAW_POSITION_KD_DEFAULTS  0

#define YAW_SPEED_KP_DEFAULTS  35
#define YAW_SPEED_KI_DEFAULTS  0
#define YAW_SPEED_KD_DEFAULTS  0

// avoid bang --->  position:20.0  speed:19.0
//big bang   22.5 20.0
#define PITCH_POSITION_KP_DEFAULTS  20.0
#define PITCH_POSITION_KI_DEFAULTS  0
#define PITCH_POSITION_KD_DEFAULTS  0

#define PITCH_SPEED_KP_DEFAULTS  19.0
#define PITCH_SPEED_KI_DEFAULTS  0
#define PITCH_SPEED_KD_DEFAULTS  0
#define CM2_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	8000,\
	&PID_Calc,\
	&PID_Reset,\
	0,\
	0,\
	0,\
}

#define CM2_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	4900,\
	&PID_Calc,\
	&PID_Reset,\
	0,\
	0,\
	0,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	25.0f,\
	0.5f,\
	20.0f,\
	0,\
	0,\
	0,\
	11000,\
	3500,\
	1500,\
	0,\
	16000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	10.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	8000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define GIMBAL_MOTOR7_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	20.0f,\
	1.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	3500,\
	1500,\
	0,\
	8000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal position pid control
//20  19
#define GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_POSITION_KP_DEFAULTS,\
	PITCH_POSITION_KI_DEFAULTS,\
	PITCH_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	5900,\
	1000,\
	1500,\
	0,\
	6000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal speed pid control
#define GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_SPEED_KP_DEFAULTS,\
	PITCH_SPEED_KI_DEFAULTS,\
	PITCH_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal yaw position pid control
#define GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_POSITION_KP_DEFAULTS,\
	YAW_POSITION_KI_DEFAULTS,\
	YAW_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	8000,\
	2000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define CHASSIS_SPEED_OFFSET_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	200,\
	0,\
	100,\
	0,\
	150,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal yaw speed pid control
#define GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_SPEED_KP_DEFAULTS,\
	YAW_SPEED_KI_DEFAULTS,\
	YAW_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	8000,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

uint32_t time_tick_1ms = 0;

RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
RampGen_t MotorRotateRamp = RAMP_GEN_DAFAULT;

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;     
PID_Regulator_t GMPSpeedPID 	 = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;			
PID_Regulator_t GMYSpeedPID 	 = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

PID_Regulator_t ShootMotorPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t ShootMotorSpeedPID = GIMBAL_MOTOR7_SPEED_PID_DEFAULT;
PID_Regulator_t Speed_Offset = CHASSIS_SPEED_OFFSET_DEFAULT;
PID_Regulator_t CAN2_CM1SpeedPID      = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//�����ӵ�3510���1
PID_Regulator_t CAN2_CM1PositionPID   = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CAN2_CM2SpeedPID      = CHASSIS_MOTOR_SPEED_PID_DEFAULT;//�����ӵ�3510���2
PID_Regulator_t CAN2_CM2PositionPID   = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

/**
������Control_Task()
���ܣ���1000HZִ�иú��������¸��������״̬,�޷���ֵ
**/
void Control_Task(void)
{
	time_tick_1ms++;									//��¼����ʱ��
	remote_error_time++;
	WorkStateFSM();										//����״̬ѡ��
	WorkStateSwitchProcess();					//��⹤��״̬�Ƿ�� ����ģʽת��PREPARE_STATE�����ǣ����³�ʼ������������
  setdistancemode();//���þ���ģʽ
//	float iii = KALMAN1.A.pData[1];
	//LLL = kalman_filter_calc(&KALMAN1, TEMP, 0);
	//KalmanPredict(&KALMAN1);
	//lll = KalmanCorrect(&KALMAN1, *TEMP);
  
//	 camera_yaw += V_SPEED/2;
	
	 //printf("%f\t",camera_yaw);
	GMYawControlLoop();								//����Y���������,0x205
//	GMPitchControlLoop();	

	//����P���������,0x206
	ShooterMControlLoop();      	//	���Ʋ���  //���������������,0x207
	{
		//����̨���������������
//		if((GetInputMode() == REMOTE_INPUT )|| (GetWorkState()== PREPARE_STATE))
//		{
			SetGimbalMotorOutput();
//		}
		
		wwdg_flag &=~0x0070;
	}
	

//	    frictionwheelmotor(800,800);
	
	//if((wwdg_flag & 0x000f) ==0x000f && time_tick_1ms%4==0)
  if(time_tick_1ms%4==0)	
	{
		CMControlLoop();								//���̿�������			 
		wwdg_flag &=~0x000f;
	}
	
	SuperiviseTask();									//�������
	//ShootSupervise();//������
		user_data_handle();//�ı��Զ������ݣ�����ϵͳ����
		data_pack_handle(0x0301,user_data,0x0013);//�û��Զ������ݴ��(������+����+����)������ϵͳ����
	  
	if(time_tick_1ms % 200 == 0)
	{
		//UART3_PrintBlock(computer_tx_buf,28);
		UART3_PrintBlock1();
	}
	{
		
	Speed_Offset.kp = 10;
	Speed_Offset.kd = 2;
	Speed_Offset.outputMax = 100; 
	
	Speed_Offset.ref = 10;
  VAL_LIMIT(robotPowerHeat.ChassisPowerBuffer,0,20);
	Speed_Offset.fdb = robotPowerHeat.ChassisPowerBuffer;
	Speed_Offset.Calc(&Speed_Offset);
	Speed_Offset.output = -Speed_Offset.output;      //Ӧ���Ǹ��Ĺ�ϵ
	VAL_LIMIT(Speed_Offset.output,-300,0);
	}
}

void ControtLoopTaskInit(void)
{
	//������ʼ��
	time_tick_1ms = 0;   //�ж��еļ�������
	//б�³�ʼ��
	GMPitchRamp.SetScale(&GMPitchRamp, MOTOR_TIME_TICK);
	GMYawRamp.SetScale(&GMYawRamp, MOTOR_TIME_TICK);
	MotorRotateRamp.SetScale(&MotorRotateRamp,MOTOR_TIME_TICK);
	 
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);
	MotorRotateRamp.ResetCounter(&MotorRotateRamp);

	//PID��ʼ��
	GMPPositionPID.Reset(&GMPPositionPID);
	GMPSpeedPID.Reset(&GMPSpeedPID);
	
	GMYPositionPID.Reset(&GMYPositionPID);
	GMYSpeedPID.Reset(&GMYSpeedPID);
	
	ShootMotorPositionPID.Reset(&ShootMotorSpeedPID);
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
void ChassisMotorInit(void)
{
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
//////////////A���ϵĿɿص�Դ����ڣ�ͨ������PH?�ܽţ������ø�Ϊ���setbits
void engineerpower_Init(void)
{		GPIO_InitTypeDef gpio;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); 
		
		gpio.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_2;
		gpio.GPIO_Mode = GPIO_Mode_OUT;//??????
		gpio.GPIO_OType = GPIO_OType_PP;//????
		gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		gpio.GPIO_PuPd = GPIO_PuPd_UP;//??
//	GPIO_StructInit(&gpio);
		GPIO_Init(GPIOH, &gpio);
	
	GPIO_SetBits(GPIOH,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_2);
}

