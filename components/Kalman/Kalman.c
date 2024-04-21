#include <stdio.h>
#include "Kalman.h"

//Math fun
#define pi  3.1415926f
#define N   2
_Bool GetMatrixInverse(float src[N][N],int n,float des[N][N]);
float getA(float arcs[N][N],int n);
void getAStart(float arcs[N][N],int n,float ans[N][N]);
//得到给定矩阵src的逆矩阵保存到des中。
_Bool GetMatrixInverse(float src[N][N],int n,float des[N][N])
{
    float flag=getA(src,n);
    float t[N][N];
    if(flag==0)
    {
        return 0;
    }
    else
    {
        getAStart(src,n,t);
        for(int i=0;i<n;i++)
        {
            for(int j=0;j<n;j++)
            {
                des[i][j]=t[i][j]/flag;
            }

        }
    }
    return 1;
}

//按第一行展开计算|A|
float getA(float arcs[N][N],int n)
{
    if(n==1)
    {
        return arcs[0][0];
    }
    float ans = 0;
    float temp[N][N];
    temp[0][0] = temp[0][1] = temp[1][0] = temp[1][1] = 0.0f;
    int i,j,k;
    for(i=0;i<n;i++)
    {
        for(j=0;j<n-1;j++)
        {
            for(k=0;k<n-1;k++)
            {
                temp[j][k] = arcs[j+1][(k>=i)?k+1:k];

            }
        }
        float t = getA(temp,n-1);
        if(i%2==0)
        {
            ans += arcs[0][i]*t;
        }
        else
        {
            ans -=  arcs[0][i]*t;
        }
    }
    return ans;
}

//计算每一行每一列的每个元素所对应的余子式，组成A*
void getAStart(float arcs[N][N],int n,float ans[N][N])
{
    if(n==1)
    {
        ans[0][0] = 1;
        return;
    }
    int i,j,k,t;
    float temp[N][N];
    for(i=0;i<n;i++)
    {
        for(j=0;j<n;j++)
        {
            for(k=0;k<n-1;k++)
            {
                for(t=0;t<n-1;t++)
                {
                    temp[k][t] = arcs[k>=i?k+1:k][t>=j?t+1:t];
                }
            }
            ans[j][i]  =  getA(temp,n-1);
            if((i+j)%2 == 1)
            {
                ans[j][i] = - ans[j][i];
            }
        }
    }
} 


//Base filter

//low Pass Filter
// void LPF_init(myLPF *LPF,float alpha)
// {
//   LPF->alpha = alpha;
//   LPF->OUT = LPF->Last_OUT = LPF->Sample = 0.0;
// }

void LPF_init(myLPF *LPF,float Ts,float fc)
{
  LPF->alpha = ( ( 2 * pi * fc * Ts ) / ( 2 * pi * fc * Ts + 1 ) );
  LPF->OUT = LPF->Last_OUT = LPF->Sample = 0.0;
}

float LPF_cal(myLPF *LPF,float Sample)
{
  LPF->Sample = Sample;
  LPF->OUT = LPF->alpha * LPF->Sample + ( 1 - LPF->alpha ) * LPF->Last_OUT;
  LPF->Last_OUT = LPF->OUT;
  return LPF->OUT;
}

//high Pass Filter
// void HPF_init(myHPF *HPF,float alpha)
// {
//   HPF->alpha = alpha;
//   HPF->OUT = HPF->Last_OUT = HPF->Sample = HPF->Last_Sample = 0.0;
// }

void HPF_init(myHPF *HPF,float Ts,float fc)
{
  HPF->alpha = ( 1 / ( 2 * pi * fc * Ts + 1 ) );
  HPF->OUT = HPF->Last_OUT = HPF->Sample = HPF->Last_Sample = 0.0;
}

float HPF_cal(myHPF *HPF,float Sample)
{
  HPF->Sample = Sample;
  HPF->OUT = HPF->alpha * HPF->Last_OUT + HPF->alpha * ( HPF->Sample - HPF->Last_Sample );
  HPF->Last_Sample = HPF->Sample;
  HPF->Last_OUT = HPF->OUT;
  return HPF->OUT;
}

//band Pass Filter

//Kalman
void Kalman_2_init(myKalman_2 *Kalman,float F_1,float F_2,float F_3,float F_4,
                                      float B_1,float B_2,
                                      float H_1,float H_2,float H_3,float H_4,
                                      float Q_1,float Q_2,float Q_3,float Q_4,
                                      float R_1,float R_2,float R_3,float R_4,
                                      float X_1,float X_2,
                                      float Ut_)
{
  Kalman->F[0][0] = F_1;
  Kalman->F[0][1] = F_2;
  Kalman->F[1][0] = F_3;
  Kalman->F[1][1] = F_4;
  Kalman->B[0] = B_1;
  Kalman->B[1] = B_2;
  Kalman->H[0][0] = H_1;
  Kalman->H[0][1] = H_2;
  Kalman->H[1][0] = H_3;
  Kalman->H[1][1] = H_4;

  Kalman->Q[0][0] = Q_1;
  Kalman->Q[0][1] = Q_2;
  Kalman->Q[1][0] = Q_3;
  Kalman->Q[1][1] = Q_4;
  Kalman->R[0][0] = R_1;
  Kalman->R[0][1] = R_2;
  Kalman->R[1][0] = R_3;
  Kalman->R[1][1] = R_4;

  Kalman->P[0][0] = 1.0;
  Kalman->P[0][1] = 0.0;
  Kalman->P[1][0] = 0.0;
  Kalman->P[1][1] = 1.0;
  Kalman->K[0][0] = 0.5;
  Kalman->K[0][1] = 0.5;
  Kalman->K[1][0] = 0.5;
  Kalman->K[1][1] = 0.5;

  Kalman->X[0] = X_1;
  Kalman->X_Pred[0] = X_1;
  Kalman->X[1] = X_2;
  Kalman->X_Pred[1] = X_2;
  Kalman->Ut = Ut_;
}

void Kalman_2_cal(myKalman_2 *Kalman,float F_1,float F_2,float F_3,float F_4,
                                     float B_1,float B_2,
                                     float Ut,
                                     float Z_1,float Z_2)
{
  Kalman->F[0][0] = F_1;
  Kalman->F[0][1] = F_2;
  Kalman->F[1][0] = F_3;
  Kalman->F[1][1] = F_4;
  Kalman->B[0] = B_1;
  Kalman->B[1] = B_2;
  Kalman->Ut = Ut;
  //1.step  Predict
  Kalman->X_Pred[0] = Kalman->F[0][0] * Kalman->X[0] + Kalman->F[0][1] * Kalman->X[1] + Kalman->B[0] * Kalman->Ut;
  Kalman->X_Pred[1] = Kalman->F[1][0] * Kalman->X[0] + Kalman->F[1][1] * Kalman->X[1] + Kalman->B[1] * Kalman->Ut;
  //2.step  Correct P Predict
  float Temp[2][2];
  Temp[0][0] = Kalman->P[0][0];
  Temp[0][1] = Kalman->P[0][1];
  Temp[1][0] = Kalman->P[1][0];
  Temp[1][1] = Kalman->P[1][1];
  float temp[2][2];
  temp[0][0] = Temp[0][0] * Kalman->F[0][0] + Temp[0][1] * Kalman->F[0][1];
  temp[0][1] = Temp[1][0] * Kalman->F[0][0] + Temp[1][1] * Kalman->F[0][1];
  temp[1][0] = Temp[0][0] * Kalman->F[1][0] + Temp[0][1] * Kalman->F[1][1];
  temp[1][1] = Temp[1][0] * Kalman->F[1][0] + Temp[1][1] * Kalman->F[1][1];
  Kalman->P[0][0] = Kalman->F[0][0] * (temp[0][0]) + 
                    Kalman->F[0][1] * (temp[0][1]) + Kalman->Q[0][0];
  Kalman->P[0][1] = Kalman->F[0][0] * (temp[1][0]) + 
                    Kalman->F[0][1] * (temp[1][1]) + Kalman->Q[0][1];
  Kalman->P[1][0] = Kalman->F[1][0] * (temp[0][0]) + 
                    Kalman->F[1][1] * (temp[0][1]) + Kalman->Q[1][0];
  Kalman->P[1][1] = Kalman->F[1][0] * (temp[1][0]) + 
                    Kalman->F[1][1] * (temp[1][1]) + Kalman->Q[1][1];
  //3.step  Cal K
  temp[0][0] = Kalman->P[0][0] * Kalman->H[0][0] + Kalman->P[0][1] * Kalman->H[0][1];
  temp[0][1] = Kalman->P[1][0] * Kalman->H[0][0] + Kalman->P[1][1] * Kalman->H[0][1];
  temp[1][0] = Kalman->P[0][0] * Kalman->H[1][0] + Kalman->P[0][1] * Kalman->H[1][1];
  temp[1][1] = Kalman->P[1][0] * Kalman->H[1][0] + Kalman->P[1][1] * Kalman->H[1][1];
  Temp[0][0] = Kalman->H[0][0] * (temp[0][0]) + 
               Kalman->H[0][1] * (temp[0][1]) + Kalman->R[0][0];
  Temp[0][1] = Kalman->H[0][0] * (temp[1][0]) + 
               Kalman->H[0][1] * (temp[1][1]) + Kalman->R[0][1];
  Temp[1][0] = Kalman->H[1][0] * (temp[0][0]) + 
               Kalman->H[1][1] * (temp[0][1]) + Kalman->R[1][0];
  Temp[1][1] = Kalman->H[1][0] * (temp[1][0]) + 
               Kalman->H[1][1] * (temp[1][1]) + Kalman->R[1][1];
  float Temp_2[2][2];
  GetMatrixInverse(Temp,2,Temp_2);
  temp[0][0] = Kalman->H[0][0] * Temp_2[0][0] + Kalman->H[1][0] * Temp_2[1][0];
  temp[0][1] = Kalman->H[0][1] * Temp_2[0][0] + Kalman->H[1][1] * Temp_2[1][0];
  temp[1][0] = Kalman->H[0][0] * Temp_2[0][1] + Kalman->H[1][0] * Temp_2[1][1];
  temp[1][1] = Kalman->H[0][1] * Temp_2[0][1] + Kalman->H[1][1] * Temp_2[1][1];
  Kalman->K[0][0] = Kalman->P[0][0] * (temp[0][0]) + 
                    Kalman->P[0][1] * (temp[0][1]);
  Kalman->K[0][1] = Kalman->P[0][0] * (temp[1][0]) + 
                    Kalman->P[0][1] * (temp[1][1]);
  Kalman->K[1][0] = Kalman->P[1][0] * (temp[0][0]) + 
                    Kalman->P[1][1] * (temp[0][1]);
  Kalman->K[1][1] = Kalman->P[1][0] * (temp[1][0]) + 
                    Kalman->P[1][1] * (temp[1][1]);
  //4.step  Cal X
  float Temp_X[2];
//   Temp_X[0] = Kalman->X[0];
//   Temp_X[1] = Kalman->X[1];
  Temp_X[0] = Kalman->X_Pred[0];
  Temp_X[1] = Kalman->X_Pred[1];
  float temp_x[2];
  temp_x[0] = Z_1 - (Kalman->H[0][0] * Temp_X[0] + Kalman->H[0][1] * Temp_X[1]);
  temp_x[1] = Z_2 - (Kalman->H[1][0] * Temp_X[1] + Kalman->H[1][1] * Temp_X[1]);
  Kalman->X[0] = Kalman->X[0] + Kalman->K[0][0] * temp_x[0] + Kalman->K[0][1] * temp_x[1];
  Kalman->X[1] = Kalman->X[1] + Kalman->K[1][0] * temp_x[0] + Kalman->K[1][1] * temp_x[1];
  //5.step  Cal P
  Temp[0][0] = Kalman->P[0][0];
  Temp[0][1] = Kalman->P[0][1];
  Temp[1][0] = Kalman->P[1][0];
  Temp[1][1] = Kalman->P[1][1];
  temp[0][0] = Kalman->H[0][0] * Temp[0][0] + Kalman->H[0][1] * Temp[1][0];
  temp[0][1] = Kalman->H[1][0] * Temp[0][0] + Kalman->H[1][1] * Temp[1][0];
  temp[1][0] = Kalman->H[0][0] * Temp[0][1] + Kalman->H[0][1] * Temp[1][1];
  temp[1][1] = Kalman->H[1][0] * Temp[0][1] + Kalman->H[1][1] * Temp[1][1];
  Kalman->P[0][0] = Kalman->P[0][0] - (Kalman->K[0][0] * (temp[0][0]) + 
                                       Kalman->K[0][1] * (temp[0][1]));
  Kalman->P[0][1] = Kalman->P[0][1] - (Kalman->K[0][0] * (temp[1][0]) + 
                                       Kalman->K[0][1] * (temp[1][1]));
  Kalman->P[1][0] = Kalman->P[1][0] - (Kalman->K[1][0] * (temp[0][0]) + 
                                       Kalman->K[1][1] * (temp[0][1]));
  Kalman->P[1][1] = Kalman->P[1][1] - (Kalman->K[1][0] * (temp[1][0]) + 
                                       Kalman->K[1][1] * (temp[1][1]));
}
