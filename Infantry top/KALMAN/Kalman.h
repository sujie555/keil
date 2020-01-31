#ifndef Kalman_H
#define Kalman_H


typedef struct 
{
	/*��������ά��*/
	int MP;
	/*״̬����ά��*/
	int DP;
	/*��������ά��*/
	int CP;

	/*Ԥ��״̬x'(k): x(k)=A*x(k-1)+B*u(k)*/
	double *state_pre;
	/*����״̬x(k): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))*/
	double *state_post;

	/*״̬���ݾ���*/
	double *transition_matrix;
	/*״̬���ݾ����ת��*/
	double transition_matrix_T[15*15];

	/*���ƾ���, ���û�п��ƣ���ʹ����*/
	double *control_matrix;
	/*��������H*/
	double *measurement_matrix;
	
	/*��������H��ת��HT*/
	double measurement_matrix_T[15*3];

	/*��������Э�������Q*/
	double *process_noise_cov;
	/*��������Э�������R*/
	double *measurement_noise_cov;
	/*��������Э�������P'(k)*/
	double *error_cov_pre;

	/*Kalman �������K(k)*/
	double *gain;
	
	/*����������Э�������P(k)*/
	double *error_cov_post;

	/*��ʱ����*/
	double temp1[15*15];
	double temp2[15*3];
	double temp3[3*3];
	double temp4[3*3];
	double temp5[3];
	double temp6[3*15];
}Kalman;

extern Kalman KALMAN1;

void InitKalman(Kalman *kalman);
void CreateKalman(Kalman *kalman);
double KalmanCorrect(Kalman* kalman, double* measurement);
void KalmanPredict(Kalman *kalman);
#endif
