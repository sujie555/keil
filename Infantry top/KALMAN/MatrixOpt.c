#include<stdlib.h>
#include<math.h>


#include "MatrixOpt.h"

#define MAX_DIM		10
//---------------------------------------------------------------------------
int max(int value1, int value2)
{
   return ((value1 > value2) ? value1 : value2);
}
//---------------------------------------------------------------------------
int min(int value1, int value2)
{
    return ((value1 < value2) ? value1 : value2);
}
//---------------------------------------------------------------------------
/*
void ones(int n, double *Y)
{
	int i, j;

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			Y[i*n+j] = 1.0;
		}
	}
}
*/
//---------------------------------------------------------------------------
double* ones(int m, int n, double *Y)
{
	int i, j;
	
	for(i = 0; i < m; i++)
	{
		for(j = 0; j < n; j++)
		{

			Y[i*n+j] = 1.0;
		}
	}

	return Y;
}
//---------------------------------------------------------------------------
void eye(int n, double *I)
{
	int i, j;

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < n; j++)
		{
			if(j == i) 
				I[i*n+j] = 1.0;
			else
				I[i*n+j] = 0.0;
		}
	}
}
//--------------------------------------------------------------------------- 
//�����ļ�
double trace(double *src, int m, int n)
{
	int i,mn;
	double sum = 0.0;

	mn = min(m, n);
	for(i = 0; i < mn; i++)
	{
		sum += src[i*n+i];
	}

    return sum;
}
//---------------------------------------------------------------------------
double det(double *a, int n)
{
    //��C�����У�����ͨ���ڴ���亯��malloc���ж�̬�ڴ���䣬ʵ��������£�
	//double *A;
	//A = (double *)malloc(sizeof(double)*n*n);

    //��C++�����У�����ͨ��new������̬�����ڴ�,ʵ��������£�
    //double *A = new double[n*n];

	//������DSP��C/C++���Գ�������У��������ö�̬�ڴ���䣬
	//���Զ���һ��ά���㹻������飬ʵ��������£�
	//double A[MAX_DIM];

	double *A;

	int ii, jj;

	double MAX = 0;
    double D = 1;
    double T = 0;
    int k, i, j = 0;
    double z = 0;

	//�����ڴ棬�ں����˳�֮ǰ��Ҫ�ͷ��ڴ�
	A = (double *)malloc(sizeof(double)*n*n);
	if(A == NULL)
	{
		//����ڴ���䲻�ɹ�������δ���
	}

    for (ii = 0; ii < n; ii++)
    {
        for (jj = 0; jj < n; jj++)
        {
            A[ii*n+jj] = a[ii*n+jj];
        }
    }

    for (k = 0; k < n; k++)
    {
        MAX = 0;
        for (i = k; i < n; i++)
        {
            T = A[i*n+k];
            if (!(T == 0))
            {
                MAX = T;
                j = i;
				//�˴�����goto calc���
				//���޸�Ϊbreak���
                // goto calc;
				break;
            }
        }

    //calc:
        if(MAX == 0)
        {
            return z;
        }
        if (j != k)
        {
            D = -D;
            for (i = k; i < n; i++)
            {
                T = A[j*n+i];
                A[j*n+i] = A[k*n+i];
                A[k*n+i] = T;
            }
        }
        for (i = k + 1; i < n; i++)
        {
            T = A[i*n+k] / MAX;
            for (j = k + 1; j < n; j++)
            {
                A[i*n+j] = A[i*n+j] - T * A[k*n+j];
            }
        }
        D = D * A[k*n+k];
    }

	if(A != NULL)
		free(A);

    return  D;
}
//---------------------------------------------------------------------------
//ͨ�þ������
//C = alpha*A + beta*B
double* GEMA(double alpha, double *A, double beta, double *B, int m, int n, double *C)
{
	int i, j;

	for(i = 0; i < m; i++)
	{	
		for(j = 0; j < n; j++)
		{
			C[i*n+j] = alpha*A[i*n+j] + beta*B[i*n+j];
		}
	}
	
	return C;	
}
//---------------------------------------------------------------------------
//�������
double* MatrixAdd(double *A, double *B, int m, int n, double *C)
{
	int i, j;

	for(i = 0; i < m; i++)
	{	
		for(j = 0; j < n; j++)
		{
			C[i*n+j] = A[i*n+j] + B[i*n+j];
		}
	}

	return C;
}

double **matrimul(double a[2][2],double b[2][1],double **c)
{
	c[0][0]=a[0][0]*b[0][0]+a[0][1]*b[1][0];
	c[1][0]=a[1][0]*b[0][0]+a[1][1]*b[1][0];
	return  c;
	
}
//---------------------------------------------------------------------------
// �������
double* MatrixSub(double *A, double *B, int m, int n, double *C)
{
	int i, j;

	for(i = 0; i < m; i++)
	{	
		for(j = 0; j < n; j++)
		{
			C[i*n+j] = A[i*n+j]-B[i*n+j];
		}
	}
	
	return C;
}
//---------------------------------------------------------------------------
// ����ת��
double* MatrixTrans(double *A, int m, int n, double *B)
{
	int i,j;

   for(i = 0; i < n; i++)
   {
		for(j = 0; j < m; j++)
		{
			B[i*m+j] = A[j*n+i];
		}
	}

	return B;
}
//---------------------------------------------------------------------------
//����ת��
/*
double* transpose(double *scr, int m, int n, double *dst)
{
	int i,j;

	for(i = 0; i < n; j++)
	{
		for(j = 0; j < m; j++)
		{
			dst[i*m+j] = scr[i*n+j];
		}
	}

	return dst;
}*/
//---------------------------------------------------------------------------
//dst = alaph*A*B + beta*C
double* GEMM(double alpha, double *A, double *B, double beta, double *C, int m, int n, int p, double *dst)
{
	int i, j, k;

	double sum;

	for(i = 0; i < m; i++)
	{
		for(j = 0; j < p; j++)
		{
			sum = 0.0;

			for(k = 0; k < n; k++)
			{
				sum += A[i*n+k]*B[k*p+j];
			}
			
			dst[i*p+j] = alpha*sum + beta*C[i*p+j];
		}
	}

	return dst;
}
//---------------------------------------------------------------------------
//dst = A*B + C
double* MatrixMulAdd(double *A, double *B, double *C, int m, int n, int p, double *dst)
{
	int i, j, k;

	double sum;

	for(i = 0; i < m; i++)
	{
		for(j = 0; j < p; j++)
		{
			sum = 0.0;

			for(k = 0; k < n; k++)
			{
				sum += A[i*n+k]*B[k*p+j];
			}
			
			dst[i*p+j] = sum + C[i*p+j];
		}
	}

	return dst;
}	
//---------------------------------------------------------------------------
//�������
double* MatrixMulti(double *A, double *B, int m, int n, int p, double *C)
{
	int i, j, k;
	double sum;

	for(i = 0; i < m; i++)
	{
		for(j = 0; j < p; j++)
		{
			sum = 0.0;

			for(k = 0; k < n; k++)
			{
				sum += A[i*n+k]*B[k*p+j];
			}
			
			C[i*p+j] = sum;
		}
	}

	return C;
}
//---------------------------------------------------------------------------
// ȫ�����
void MatrixZero(double *A, int m, int n)
{
	int i,j;

    for(i = 0; i < m; i++)
	{
		for(j = 0; j < n; j++)
		{
			A[i*n+j] = 0.0;
		}
	}
}
//---------------------------------------------------------------------------
// ��λ����
/*void MatrixOnes(double *A,int n)
{
	int i;

	for(i = 0; i < n; i++)
	{
		A[i*n+i] = 1.0;
	}
}*/
//---------------------------------------------------------------------------
// �ԽǾ��󣬶Խ����ϵ�Ԫ��������Vָ��
/*
void MatrixDiag(double *A, double *V, int n)
{
	int i;

	for(i = 0; i < n; i++)
	{
		A[i*n+i] = V[i];
	}
}
*/
//---------------------------------------------------------------------------
// ������Գ��� B = const * A
double* MatrixMultiFactor(double alpha, double *A, int m, int n, double *B)
{
	int i,j;

	for(i = 0; i < m; i++)
	{
		for(j = 0; j < n; j++)
		{
			B[i*n+j] = alpha*A[i*n+j];
		}
	}

	return B;
}
//---------------------------------------------------------------------------
double* MatrixTimes(double cnst, double *A, int m, int n, double *B)
{
	int i,j;

	for(i = 0; i < m; i++)
	{
		for(j = 0; j < n; j++)
		{
			B[i*n+j] = cnst*A[i*n+j];
		}
	}

	return B;
}
//---------------------------------------------------------------------------
// ����ֵ
/*
void MatrixEval(double *dest, double *src, int m, int n)
{
	int i, j;

	for(i = 0; i < m; i++)
	{
		for(j = 0; j < n; j++)
		{
			dest[i*n+j] = src[i*n+j];
		}
	}
}
*/
//---------------------------------------------------------------------------
// 2�׷�������
void Mat2Inv(double *src, double *dst)
{
	double c11, c12;
	double c21, c22;
	double c;
	
	c11=src[0]; c12=src[1];
	c21=src[2]; c22=src[3];
	
	c=(c11*c22 - c12*c21);
	
	dst[0]= c22/c;
	dst[1]=-c12/c;
	dst[2]=-c21/c;
	dst[3]= c11/c;
}
//---------------------------------------------------------------------------
// 3�׷�������
void Mat3Inv(double *src, double *dst)
{
	double c11, c12, c13;
	double c21, c22, c23;
	double c31, c32, c33;
	double  c;

	c11=src[0]; c12=src[1];c13=src[2];
	c21=src[3]; c22=src[4];c23=src[5];
	c31=src[6]; c32=src[7];c33=src[8];

	c = (c11*c22*c33 - c11*c23*c32 - c12*c21*c33 + c12*c23*c31 + c13*c21*c32 - c13*c22*c31);

	dst[0]= (c22*c33 - c23*c32)/c;
	dst[1]=-(c12*c33 - c13*c32)/c;
	dst[2]= (c12*c23 - c13*c22)/c;
	dst[3]=-(c21*c33 - c23*c31)/c;
	dst[4]= (c11*c33 - c13*c31)/c;
	dst[5]=-(c11*c23 - c13*c21)/c;
	dst[6]= (c21*c32 - c22*c31)/c;
	dst[7]=-(c11*c32 - c12*c31)/c;
	dst[8]= (c11*c22 - c12*c21)/c;
}
//---------------------------------------------------------------------------
// ��3��3ά��ʸ����ϳ�һ��3��3�׵ľ���
void make_mat(double *v1, double *v2, double *v3, double *mn, int flag)
{
	int i;
	
	// flag = 1,��ʾ���й��ɾ��󣬼�mn = [v1';v2';v3'];
	if(flag == 1)
	{
		for(i = 0; i < 3; i++)
		{
			mn[0*3+i] = v1[i];
			mn[1*3+i] = v2[i];
			mn[2*3+i] = v3[i];
		}
	}
	else if(flag == 2)
	{
		for(i=0; i < 3; i++)
		{
			mn[i*3+0] = v1[i];
			mn[i*3+1] = v2[i];
			mn[i*3+2] = v3[i];
		}
	}
}
//---------------------------------------------------------------------------
void MatrixInvert(double *A, int n, double *B)
{
    double e, y, det, w, d = 0, d1;
    int i, j, k, ir, ip;

    //int *jz = new int[n];
	//int jz[MAX_DIM];
	int *jz;

    //double *c = new double[n];
	//double c[MAX_DIM];
	double *c;

    //double *ab = new double[n];
	//double ab[MAX_DIM];
	double *ab;

	//��̬�����ڴ�
	jz = (int *)malloc(sizeof(int)*n);
	c = (double *)malloc(sizeof(double)*n);
	ab = (double *)malloc(sizeof(double)*n);
	
	if(jz == NULL || c == NULL || ab == NULL)
	{
		//�ڴ����ʧ�ܣ���δ���
	}
	
    for (i = 0; i < n; i++)
    {
		for (j = 0; j < n; j++)
		{

            B[i*n+j] = A[i*n+j];
	    }
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            e = fabs(B[i*n+j]);
            if (d < e) d = e;
        }
    }

    d1 = 1 / d;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            B[i*n+j] *= d1;
        }
    }

    e = 1.0e-26;
    det=1;

    for (i = 0; i < n;  i++)
    {
		jz[i]=i;
	}

    for (i = 0; i < n; i++)
    {
        k = i;
        y = B[i*n+i];
        ir = i - 1;
        ip = i + 1;
        if (ip < n)
        {
            for (j = ip; j < n; j++)
            {
                w = B[i*n+j];
                if (fabs(w) > fabs(y))
                {
                    k = j;
                    y = w;
                }
            }
        }

        det *= y;
        y = 1.0 / y;

        for (j = 0; j < n; j++)
        {
            c[j] = B[j*n+k];
            B[j*n+k] = B[j*n+i];

            B[j*n+i] = -c[j] * y;
            ab[j] = B[i*n+j] * y;

            B[i*n+j] = ab[j];
        }

        B[i*n+i] = y;
        j = jz[i];
        jz[i] = jz[k];
        jz[k] = j;
        k = 0;
        do
        {
            if((k <= ir) | (k >= ip))
            {
                j = 0;
                do
                {
                    if ((j <= ir) | (j >= ip))
                    {
                        B[k*n+j] -= ab[j] * c[k];
                    }
                    j++;
                }while (j < n);
            }
            k++;
        }while(k < n);
    }
    i = 0;
    do
    {
        k = jz[i];
        if (k != i)
        {
            for(j = 0; j < n; j++)
            {
                w = B[i*n+j];
                B[i*n+j] = B[k*n+j];
                B[k*n+j] = w;
            }
            ip = jz[i];
            jz[i] = jz[k];
            jz[k] = ip;
            det = -det;
        }
        else
        {
            i++;
        }
    }while (i < n);

    d1= 1.0 / d;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            B[i*n+j] *= d1;
        }
    }

	//�ͷ��ڴ�ռ�
	if(jz != NULL) 
		free(jz);

	if(c != NULL)
		free(c);

	if(ab != NULL)
		free(ab);
}
//---------------------------------------------------------------------------
