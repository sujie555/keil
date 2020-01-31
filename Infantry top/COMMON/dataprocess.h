#ifndef __DATAPROCESS_H__
#define __DATAPROCESS_H__

#include "main.h"
#include "arm_math.h"

#define BUFFER_MAX   100

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

typedef __packed struct
{
	uint8_t sof;             //����֡ͷ���̶�0xA5
	uint16_t data_length;    //ÿ֡������data�ĳ��� 
	uint8_t seq;             //�����
	uint8_t crc8;            //֡ͷ��crcУ����
}frame_header_t;

typedef __packed struct
{
	uint8_t robot_id;
	uint16_t stageRemainTime; //��ǰ�׶�ʣ��ʱ��
	uint8_t gameProgress;     //��ǰ���������ĸ��׶�
	uint8_t robotLevel;       //�����˵�ǰ�ȼ�
	uint16_t remainHP;        //�����˵�ǰѪ��
	uint16_t maxHP;           //��������Ѫ��
}extGameRobotState_t;  //����������״̬

typedef __packed struct
{
	uint8_t armorType; //��װ�ױ������Ѫʱ����ʾ�ܵ��˺���װ��ID
	uint8_t hurtType;  //Ѫ���仯����
}extRobotHurt_t; //�˺�����

typedef __packed struct
{
	uint16_t ChassisVolt;        //���������ѹ
	uint16_t ChassisCurrent;     //�����������
	float ChassisPower;       //�����������
	uint16_t ChassisPowerBuffer; //���̹��ʻ���
	uint16_t shooter_17_Heat;   //17mmǹ������
	uint16_t shooter_42_Heat;   //42mmǹ������
}extPowerHeatData_t;  //ʵʱ������������

typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}extShowData_t;  //�������Զ�������

typedef __packed struct
{
	int headPosition;
	int tailPosition;
	uint8_t ringBuf[BUFFER_MAX];
}ringBuffer_t;

typedef __packed struct
{
	uint8_t bulletType; //��������  1��17mm����   2��42mm����
	uint8_t bulletFreq; //������Ƶ
	float bulletSpeed;  //��������
}extShootData_t;  //ʵʱ�����Ϣ

typedef struct
{
  float raw_value;
  float filtered_value[1];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
	float filtered_value[1];
	float xhat_data[2], xhatminus_data[2], z_data[1], Pminus_data[4], K_data[2];
	float P_data[4];
	float AT_data[4], HT_data[2];
	float A_data[4];
	float H_data[2];
	float Q_data[4];
	float R_data[1];
} kalman_filter_init_t;



void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);

extern ringBuffer_t buffer;
extern uint16_t Heat17_Max;
extern extPowerHeatData_t  robotPowerHeat;
extern extGameRobotState_t robotState;
extern uint8_t computer_tx_buf[28];
extern uint8_t user_data[19];
extern uint16_t bulletCounter;
extern extShootData_t robotShootData;
extern float chassisPowerError;
extern float chassisPowerBuffer;
void getRobotState(uint8_t *stateData);
void getRobotPowerHeat(uint8_t *powerHeartData);
void getRobotShootData(uint8_t *shootData);

float Parameter_Transformation(int32_t data);
void RingBuffer_Write(uint8_t data);
void user_data_handle(void);
u8   RingBuffer_Read(uint8_t *pdata);

#endif
