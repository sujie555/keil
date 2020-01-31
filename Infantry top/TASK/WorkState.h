#ifndef __WORKSTATE_H__
#define __WORKSTATE_H__
#include <stm32f4xx.h>

typedef enum
{
    PREPARE_STATE,     		//�ϵ���ʼ��״̬ 4s������
//    STANDBY_STATE,				//������״̬
    NORMAL_STATE,					//�����ֿ���״̬
		CAMERA_STATE,    				//���״̬
	  STOP_STATE,        		//ֹͣ�˶�״̬
    ADJUST_STATE,    				//����״̬��������׼ 
	  IDENTIFY_STATE,         //ս��ʶ��ģʽ
	  CATWALK_STATE,        //è��ģʽ
}WorkState_e;


extern uint8_t camera_temp;
extern uint8_t cat_temp;
extern uint8_t dafu_temp;

extern int flag_prepare_ready;
extern int flag_Dafu_fromsight;

void WorkStateFSM(void);
void WorkStateSwitchProcess(void);
void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);
WorkState_e GetlastWorkState(void);
void AllDataInit(void);

#endif
