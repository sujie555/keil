#ifndef __USART3_H__
#define __USART3_H__

#define REMOTE_DMA1_RX_BUF_LEN    100
#define Robot_State_Rx_Len        17          //������״̬���ݣ�Ӧ���ܵ�������
#define Robot_Hurt_Rx_Len         10          //�������˺����ݣ�Ӧ���ܵ�������
#define Robot_PowerHeat_Rx_Len    29          //�����˹����������ݣ�Ӧ���ܵ�������
#define Robot_Shoot_Rx_Len        15          //������������ݣ�Ӧ���ܵ�������
void USART3_Init(void);
extern uint8_t REMOTE_DMA1_RX_BUF[2][REMOTE_DMA1_RX_BUF_LEN];
void UART3_PrintBlock(uint8_t* pdata, uint8_t len);//���鷢��
void UART3_PrintBlock1(void);
#endif
