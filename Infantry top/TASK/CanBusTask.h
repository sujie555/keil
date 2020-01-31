#ifndef __CANBUSTASK_H__
#define __CANBUSTASK_H__
#include <stm32f4xx.h>

/* Chassis Motor */
#define CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS1_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS1_MOTOR4_FEEDBACK_MSG_ID           0x204
/* Gimbal Motor  */
#define CAN_BUS1_MOTOR5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS1_MOTOR6_FEEDBACK_MSG_ID           0x206

#define CAN_BUS1_MOTOR7_FEEDBACK_MSG_ID           0x207
#define RATE_BUF_SIZE 6
/*****************CAN2_Motor****************/
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID                  0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID                  0x202 
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID                  0x205
#define CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID                  0x207


typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ,�������Ϊ�ٶ�
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//����RATE_BUF_SIZE�ε�diff
	int32_t round_cnt;										//��¼Ȧ��
	float filter_rate;											//�ٶ�,��diff�RATE_BUF_SIZE��ƽ��
	double ecd_angle;											//�Ƕ�
	double ecd_xtl_angle;
  float real_torque_current;          //ʵ��ת�ص���
}Encoder;

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder CM7Encoder;
extern float ZGyroModuleAngle;
extern volatile Encoder CAN2_CM1Encoder;
extern volatile Encoder CAN2_CM2Encoder;
extern volatile Encoder CAN2_CM5Encoder;
extern volatile Encoder CAN2_CM7Encoder;

void CanReceiveMsgProcess(CanRxMsg * msg);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void Can2_ReceiveMsgProcess(CanRxMsg * msg);
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t cm7_iq);
void Set_Gimbal_CALI_STATE(CAN_TypeDef *CANx);
#endif
