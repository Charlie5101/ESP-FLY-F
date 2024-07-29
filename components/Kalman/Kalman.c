#include <stdio.h>
#include "Kalman.h"

//Math fun
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

void imu_kalman_init(imu_Kalman *Kalman,float t,
                                        float Q_0_0,float Q_0_1,float Q_1_0,float Q_1_1,
                                        float Q_2_2,float Q_2_3,float Q_3_2,float Q_3_3,
                                        float Q_4_4,float Q_4_5,float Q_5_4,float Q_5_5,

                                        float R_0,float R_1,float R_2);
void imu_kalman_cal(imu_Kalman *Kalman, float A_0_0,float A_0_1,float A_1_0,float A_1_1,
                                        float A_2_2,float A_2_3,float A_3_2,float A_3_3,
                                        float A_4_4,float A_4_5,float A_5_4,float A_5_5,

                                        float B_0_0,float B_1_0,
                                        float B_2_1,float B_3_1,
                                        float B_4_2,float B_5_2,

                                        float U_1,float U_2,float U_3,
                                        float Z_1,float Z_2,float Z_3);
void imu_kalman_cal_d(imu_Kalman *Kalman, float t,
                                          float U_roll,float U_pitch,float U_yaw,
                                          float Z_droll,float Z_dpitch,float Z_dyaw);
void imu_kalman_d2_init(imu_Kalman *Kalman,float t,
                                        float Q_0_0,float Q_0_1,float Q_1_0,float Q_1_1,
                                        float Q_2_2,float Q_2_3,float Q_3_2,float Q_3_3,
                                        float Q_4_4,float Q_4_5,float Q_5_4,float Q_5_5,

                                        float R_0_0,float R_0_1,float R_1_0,float R_1_1,
                                        float R_2_2,float R_2_3,float R_3_2,float R_3_3,
                                        float R_4_4,float R_4_5,float R_5_4,float R_5_5);
void imu_kalman_cal_d2(imu_Kalman *Kalman, float t,
                                          float U_roll,float U_pitch,float U_yaw,
                                          float Z_droll,float Z_dpitch,float Z_dyaw,
                                          float Z_roll,float Z_pitch,float Z_yaw);


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

void imu_kalman_init(imu_Kalman *Kalman,float t,
                                        float Q_0_0,float Q_0_1,float Q_1_0,float Q_1_1,
                                        float Q_2_2,float Q_2_3,float Q_3_2,float Q_3_3,
                                        float Q_4_4,float Q_4_5,float Q_5_4,float Q_5_5,

                                        float R_0,float R_1,float R_2)
{
  //A
  Kalman->A[0][0] = 1.0;
  Kalman->A[0][1] = t;
  Kalman->A[1][0] = 0;
  Kalman->A[1][1] = 1.0;

  Kalman->A[2][2] = 1.0;
  Kalman->A[2][3] = t;
  Kalman->A[3][2] = 0;
  Kalman->A[3][3] = 1.0;

  Kalman->A[4][4] = 1.0;
  Kalman->A[4][5] = t;
  Kalman->A[5][4] = 0;
  Kalman->A[5][5] = 1.0;
  //B
  Kalman->B[0][0] = 0.5 * t * t;
  Kalman->B[1][0] = t;
  Kalman->B[2][1] = 0.5 * t * t;
  Kalman->B[3][1] = t;
  Kalman->B[4][2] = 0.5 * t * t;
  Kalman->B[5][2] = t;
  //Q
  Kalman->Q[0][0] = Q_0_0;
  Kalman->Q[0][1] = Q_0_1;
  Kalman->Q[1][0] = Q_1_0;
  Kalman->Q[1][1] = Q_1_1;

  Kalman->Q[2][2] = Q_2_2;
  Kalman->Q[2][3] = Q_2_3;
  Kalman->Q[3][2] = Q_3_2;
  Kalman->Q[3][3] = Q_3_3;

  Kalman->Q[4][4] = Q_4_4;
  Kalman->Q[4][5] = Q_4_5;
  Kalman->Q[5][4] = Q_5_4;
  Kalman->Q[5][5] = Q_5_5;
  //R
  Kalman->R[0][0] = R_0;

  Kalman->R[1][1] = R_1;

  Kalman->R[2][2] = R_2;
  //H
  Kalman->H[0][0] = 0;
  Kalman->H[0][1] = 1.0;

  Kalman->H[1][2] = 0;
  Kalman->H[1][3] = 1.0;

  Kalman->H[2][4] = 0;
  Kalman->H[2][5] = 1.0;
  //P
  Kalman->P[0][0] = 1.0;
  Kalman->P[0][1] = 0;
  Kalman->P[1][0] = 0;
  Kalman->P[1][1] = 1.0;

  Kalman->P[2][2] = 1.0;
  Kalman->P[2][3] = 0;
  Kalman->P[3][2] = 0;
  Kalman->P[3][3] = 1.0;

  Kalman->P[4][4] = 1.0;
  Kalman->P[4][5] = 0;
  Kalman->P[5][4] = 0;
  Kalman->P[5][5] = 1.0;
  //K
  Kalman->K[0][0] = 0.5;
  Kalman->K[1][0] = 0.5;
  Kalman->K[2][1] = 0.5;
  Kalman->K[3][1] = 0.5;
  Kalman->K[4][2] = 0.5;
  Kalman->K[5][2] = 0.5;
}

void imu_kalman_cal(imu_Kalman *Kalman, float A_0_0,float A_0_1,float A_1_0,float A_1_1,
                                        float A_2_2,float A_2_3,float A_3_2,float A_3_3,
                                        float A_4_4,float A_4_5,float A_5_4,float A_5_5,

                                        float B_0_0,float B_1_0,
                                        float B_2_1,float B_3_1,
                                        float B_4_2,float B_5_2,

                                        float U_1,float U_2,float U_3,
                                        float Z_1,float Z_2,float Z_3)
{
  //step 1

  //step 2
}

void imu_kalman_cal_d(imu_Kalman *Kalman, float t,
                                          float U_roll,float U_pitch,float U_yaw,
                                          float Z_droll,float Z_dpitch,float Z_dyaw)
{
  //step 1
  float Temp_X[6][3] = {0};
  Temp_X[0][0] = 0.5 * U_roll * t * t + Kalman->X_hat[1][0] * t + Kalman->X_hat[0][0];      //0.5 * R2 * t * t + R1 * t + R0;
  Temp_X[1][0] = U_roll * t + Kalman->X_hat[1][0];                                          //R2 * t + R1;

  Temp_X[2][1] = 0.5 * U_pitch * t * t + Kalman->X_hat[3][1] * t + Kalman->X_hat[2][1];    //0.5 * P2 * t * t + P1 * t + P0;
  Temp_X[3][1] = U_pitch * t + Kalman->X_hat[3][1];                                        //P2 * t + P1;

  Temp_X[4][2] = 0.5 * U_yaw * t * t + Kalman->X_hat[5][2] * t + Kalman->X_hat[4][2];      //0.5 * Y2 * t * t + Y1 * t + Y0;
  Temp_X[5][2] = U_yaw * t + Kalman->X_hat[5][2];                                          //Y2 * t + Y1;

  // memcpy()
  Kalman->X_hat[0][0] = Temp_X[0][0];
  Kalman->X_hat[1][0] = Temp_X[1][0];
  Kalman->X_hat[2][1] = Temp_X[2][1];
  Kalman->X_hat[3][1] = Temp_X[3][1];
  Kalman->X_hat[4][2] = Temp_X[4][2];
  Kalman->X_hat[5][2] = Temp_X[5][2];
  //step 2
  float Temp_P[6][6] = {0};
  Temp_P[0][0] = Kalman->P[1][1] * t * t + Kalman->P[1][0] * t + Kalman->P[0][1] * t + Kalman->P[0][0] + Kalman->Q[0][0];
                                                                                          //P11 * t * t + P10 * t + P1 * t + P0 + Q0;
  Temp_P[0][1] = Kalman->P[1][1] * t + Kalman->P[0][1] + Kalman->Q[0][1];                 //P11 * t + P1 + Q1;
  Temp_P[1][0] = Kalman->P[1][1] * t + Kalman->P[1][0] + Kalman->Q[1][0];                 //P11 * t + P10 + Q10;
  Temp_P[1][1] = Kalman->P[1][1] + Kalman->Q[1][1];                                       //P11 + Q11;

  Temp_P[2][2] = Kalman->P[3][3] * t * t + Kalman->P[3][2] * t + Kalman->P[2][3] * t + Kalman->P[2][2] + Kalman->Q[2][2];
                                                                                          //P33 * t * t + P32 * t + P23 * t + P22 + Q22;
  Temp_P[2][3] = Kalman->P[3][3] * t + Kalman->P[2][3] + Kalman->Q[2][3];                 //P33 * t + P23 + Q23;
  Temp_P[3][2] = Kalman->P[3][3] * t + Kalman->P[3][2] + Kalman->Q[3][2];                 //P33 * t + P32 + Q32;
  Temp_P[3][3] = Kalman->P[3][3] + Kalman->Q[3][3];                                       //P33 + Q33;

  Temp_P[4][4] = Kalman->P[5][5] * t * t + Kalman->P[5][4] * t + Kalman->P[4][5] * t + Kalman->P[4][4] + Kalman->Q[4][4];
                                                                                          //P55 * t * t + P54 * t + P45 * t + P44 + Q44;
  Temp_P[4][5] = Kalman->P[5][5] * t + Kalman->P[4][5] + Kalman->Q[4][5];                 //P55 * t + P45 + Q45;
  Temp_P[5][4] = Kalman->P[5][5] * t + Kalman->P[5][4] + Kalman->Q[5][4];                 //P55 * t + P54 + Q54;
  Temp_P[5][5] = Kalman->P[5][5] + Kalman->Q[5][5];                                       //P55 + Q55;

  // memcpy()
  Kalman->P[0][0] = Temp_P[0][0];
  Kalman->P[0][1] = Temp_P[0][1];
  Kalman->P[1][0] = Temp_P[1][0];
  Kalman->P[1][1] = Temp_P[1][1];

  Kalman->P[2][2] = Temp_P[2][2];
  Kalman->P[2][3] = Temp_P[2][3];
  Kalman->P[3][2] = Temp_P[3][2];
  Kalman->P[3][3] = Temp_P[3][3];

  Kalman->P[4][4] = Temp_P[4][4];
  Kalman->P[4][5] = Temp_P[4][5];
  Kalman->P[5][4] = Temp_P[5][4];
  Kalman->P[5][5] = Temp_P[5][5];
  //step 3
  Kalman->K[0][0] = Kalman->P[0][1] / ( Kalman->P[1][1] + Kalman->R[0][0] );
  Kalman->K[1][0] = Kalman->P[1][1] / ( Kalman->P[1][1] + Kalman->R[0][0] );

  Kalman->K[2][1] = Kalman->P[2][3] / ( Kalman->P[3][3] + Kalman->R[1][1] );
  Kalman->K[3][1] = Kalman->P[3][3] / ( Kalman->P[3][3] + Kalman->R[1][1] );

  Kalman->K[4][2] = Kalman->P[4][5] / ( Kalman->P[5][5] + Kalman->R[2][2] );
  Kalman->K[5][2] = Kalman->P[5][5] / ( Kalman->P[5][5] + Kalman->R[2][2] );
  //step 4
  Temp_X[0][0] = Kalman->K[0][0] * Z_droll - Kalman->K[0][0] * Kalman->X_hat[1][0] + Kalman->X_hat[0][0];      //K0 * ZR - K0 * R1 + R0;
  Temp_X[1][0] = Kalman->K[1][0] * Z_droll - Kalman->K[1][0] * Kalman->X_hat[1][0] + Kalman->X_hat[1][0];      //K10 * ZR - K10 * R1 + R1;

  Temp_X[2][1] = Kalman->K[2][1] * Z_dpitch - Kalman->K[2][1] * Kalman->X_hat[3][1] + Kalman->X_hat[2][1];     //K21 * ZP - K21 * P1 + P0;
  Temp_X[3][1] = Kalman->K[3][1] * Z_dpitch - Kalman->K[3][1] * Kalman->X_hat[3][1] + Kalman->X_hat[3][1];     //K31 * ZP - K31 * P1 + P1;

  Temp_X[4][2] = Kalman->K[4][2] * Z_dyaw - Kalman->K[4][2] * Kalman->X_hat[5][2] + Kalman->X_hat[4][2];       //K42 * ZY - K42 * Y1 + Y0;
  Temp_X[5][2] = Kalman->K[5][2] * Z_dyaw - Kalman->K[5][2] * Kalman->X_hat[5][2] + Kalman->X_hat[5][2];       //K52 * ZY - K52 * Y1 + Y1;

  // memcpy()
  Kalman->X_hat[0][0] = Temp_X[0][0];
  Kalman->X_hat[1][0] = Temp_X[1][0];
  Kalman->X_hat[2][1] = Temp_X[2][1];
  Kalman->X_hat[3][1] = Temp_X[3][1];
  Kalman->X_hat[4][2] = Temp_X[4][2];
  Kalman->X_hat[5][2] = Temp_X[5][2];
  //step 5
  Temp_P[0][0] = Kalman->P[0][0] - Kalman->K[0][0]  * Kalman->P[1][0];                    //P0 - K0 * P10;
  Temp_P[0][1] = Kalman->P[0][1] - Kalman->K[0][0]  * Kalman->P[1][1];                    //P1 - K0 * P11;
  Temp_P[1][0] = Kalman->P[1][0] - Kalman->K[0][0]  * Kalman->P[1][0];                    //P10 - K0 * P10;
  Temp_P[1][1] = Kalman->P[1][1] - Kalman->K[0][0]  * Kalman->P[1][1];                    //P11 - K0 * P11;

  Temp_P[2][2] = Kalman->P[2][2] - Kalman->K[2][1]  * Kalman->P[3][2];                    //P22 - K21 * P32;
  Temp_P[2][3] = Kalman->P[2][3] - Kalman->K[2][1]  * Kalman->P[3][3];                    //P23 - K21 * P33;
  Temp_P[3][2] = Kalman->P[3][2] - Kalman->K[3][1]  * Kalman->P[3][2];                    //P32 - K31 * P32;
  Temp_P[3][3] = Kalman->P[3][3] - Kalman->K[3][1]  * Kalman->P[3][3];                    //P33 - K31 * P33;

  Temp_P[4][4] = Kalman->P[4][4] - Kalman->K[4][2]  * Kalman->P[5][4];                    //P44 - K42 * P54;
  Temp_P[4][5] = Kalman->P[4][5] - Kalman->K[4][2]  * Kalman->P[5][5];                    //P45 - K42 * P55;
  Temp_P[5][4] = Kalman->P[5][4] - Kalman->K[5][2]  * Kalman->P[5][4];                    //P54 - K52 * P54;
  Temp_P[5][5] = Kalman->P[5][5] - Kalman->K[5][2]  * Kalman->P[5][5];                    //P55 - K52 * P55;
  // memcpy()
  Kalman->P[0][0] = Temp_P[0][0];
  Kalman->P[0][1] = Temp_P[0][1];
  Kalman->P[1][0] = Temp_P[1][0];
  Kalman->P[1][1] = Temp_P[1][1];

  Kalman->P[2][2] = Temp_P[2][2];
  Kalman->P[2][3] = Temp_P[2][3];
  Kalman->P[3][2] = Temp_P[3][2];
  Kalman->P[3][3] = Temp_P[3][3];

  Kalman->P[4][4] = Temp_P[4][4];
  Kalman->P[4][5] = Temp_P[4][5];
  Kalman->P[5][4] = Temp_P[5][4];
  Kalman->P[5][5] = Temp_P[5][5];
}

void imu_kalman_d2_init(imu_Kalman *Kalman,float t,
                                        float Q_0_0,float Q_0_1,float Q_1_0,float Q_1_1,
                                        float Q_2_2,float Q_2_3,float Q_3_2,float Q_3_3,
                                        float Q_4_4,float Q_4_5,float Q_5_4,float Q_5_5,

                                        float R_0_0,float R_0_1,float R_1_0,float R_1_1,
                                        float R_2_2,float R_2_3,float R_3_2,float R_3_3,
                                        float R_4_4,float R_4_5,float R_5_4,float R_5_5)
{
  //A
  Kalman->A[0][0] = 1.0;
  Kalman->A[0][1] = t;
  Kalman->A[1][0] = 0;
  Kalman->A[1][1] = 1.0;

  Kalman->A[2][2] = 1.0;
  Kalman->A[2][3] = t;
  Kalman->A[3][2] = 0;
  Kalman->A[3][3] = 1.0;

  Kalman->A[4][4] = 1.0;
  Kalman->A[4][5] = t;
  Kalman->A[5][4] = 0;
  Kalman->A[5][5] = 1.0;
  //B
  Kalman->B[0][0] = 0.5 * t * t;
  Kalman->B[1][0] = t;
  Kalman->B[2][1] = 0.5 * t * t;
  Kalman->B[3][1] = t;
  Kalman->B[4][2] = 0.5 * t * t;
  Kalman->B[5][2] = t;
  //Q
  Kalman->Q[0][0] = Q_0_0;
  Kalman->Q[0][1] = Q_0_1;
  Kalman->Q[1][0] = Q_1_0;
  Kalman->Q[1][1] = Q_1_1;

  Kalman->Q[2][2] = Q_2_2;
  Kalman->Q[2][3] = Q_2_3;
  Kalman->Q[3][2] = Q_3_2;
  Kalman->Q[3][3] = Q_3_3;

  Kalman->Q[4][4] = Q_4_4;
  Kalman->Q[4][5] = Q_4_5;
  Kalman->Q[5][4] = Q_5_4;
  Kalman->Q[5][5] = Q_5_5;
  //R
  Kalman->R[0][0] = R_0_0;
  Kalman->R[0][1] = R_0_1;
  Kalman->R[1][0] = R_1_0;
  Kalman->R[1][1] = R_1_1;

  Kalman->R[2][2] = R_2_2;
  Kalman->R[2][3] = R_2_3;
  Kalman->R[3][2] = R_3_2;
  Kalman->R[3][3] = R_3_3;

  Kalman->R[4][4] = R_4_4;
  Kalman->R[4][5] = R_4_5;
  Kalman->R[5][4] = R_5_4;
  Kalman->R[5][5] = R_5_5;
  //H
  Kalman->H[0][0] = 0;
  Kalman->H[0][1] = 1.0;

  Kalman->H[1][2] = 0;
  Kalman->H[1][3] = 1.0;

  Kalman->H[2][4] = 0;
  Kalman->H[2][5] = 1.0;
  //P
  Kalman->P[0][0] = 1.0;
  Kalman->P[0][1] = 0;
  Kalman->P[1][0] = 0;
  Kalman->P[1][1] = 1.0;

  Kalman->P[2][2] = 1.0;
  Kalman->P[2][3] = 0;
  Kalman->P[3][2] = 0;
  Kalman->P[3][3] = 1.0;

  Kalman->P[4][4] = 1.0;
  Kalman->P[4][5] = 0;
  Kalman->P[5][4] = 0;
  Kalman->P[5][5] = 1.0;
  //K
  Kalman->K[0][0] = 0.5;
  Kalman->K[1][0] = 0.5;
  Kalman->K[2][1] = 0.5;
  Kalman->K[3][1] = 0.5;
  Kalman->K[4][2] = 0.5;
  Kalman->K[5][2] = 0.5;
}

void imu_kalman_cal_d2(imu_Kalman *Kalman, float t,
                                          float U_roll,float U_pitch,float U_yaw,
                                          float Z_droll,float Z_dpitch,float Z_dyaw,
                                          float Z_roll,float Z_pitch,float Z_yaw)
{
  //step 1
  static float Temp_X[6][3] = {0};
  Temp_X[0][0] = 0.5 * U_roll * t * t + Kalman->X_hat[1][0] * t + Kalman->X_hat[0][0];      //0.5 * R2 * t * t + R1 * t + R0;
  Temp_X[1][0] = U_roll * t + Kalman->X_hat[1][0];                                          //R2 * t + R1;

  Temp_X[2][1] = 0.5 * U_pitch * t * t + Kalman->X_hat[3][1] * t + Kalman->X_hat[2][1];    //0.5 * P2 * t * t + P1 * t + P0;
  Temp_X[3][1] = U_pitch * t + Kalman->X_hat[3][1];                                        //P2 * t + P1;

  Temp_X[4][2] = 0.5 * U_yaw * t * t + Kalman->X_hat[5][2] * t + Kalman->X_hat[4][2];      //0.5 * Y2 * t * t + Y1 * t + Y0;
  Temp_X[5][2] = U_yaw * t + Kalman->X_hat[5][2];                                          //Y2 * t + Y1;

  // memcpy()
  Kalman->X_hat[0][0] = Temp_X[0][0];
  Kalman->X_hat[1][0] = Temp_X[1][0];
  Kalman->X_hat[2][1] = Temp_X[2][1];
  Kalman->X_hat[3][1] = Temp_X[3][1];
  Kalman->X_hat[4][2] = Temp_X[4][2];
  Kalman->X_hat[5][2] = Temp_X[5][2];
  //step 2
  static float Temp_P[6][6] = {0};
  Temp_P[0][0] = Kalman->P[1][1] * t * t + Kalman->P[1][0] * t + Kalman->P[0][1] * t + Kalman->P[0][0] + Kalman->Q[0][0];
                                                                                          //P11 * t * t + P10 * t + P1 * t + P0 + Q0;
  Temp_P[0][1] = Kalman->P[1][1] * t + Kalman->P[0][1] + Kalman->Q[0][1];                 //P11 * t + P1 + Q1;
  Temp_P[1][0] = Kalman->P[1][1] * t + Kalman->P[1][0] + Kalman->Q[1][0];                 //P11 * t + P10 + Q10;
  Temp_P[1][1] = Kalman->P[1][1] + Kalman->Q[1][1];                                       //P11 + Q11;

  Temp_P[2][2] = Kalman->P[3][3] * t * t + Kalman->P[3][2] * t + Kalman->P[2][3] * t + Kalman->P[2][2] + Kalman->Q[2][2];
                                                                                          //P33 * t * t + P32 * t + P23 * t + P22 + Q22;
  Temp_P[2][3] = Kalman->P[3][3] * t + Kalman->P[2][3] + Kalman->Q[2][3];                 //P33 * t + P23 + Q23;
  Temp_P[3][2] = Kalman->P[3][3] * t + Kalman->P[3][2] + Kalman->Q[3][2];                 //P33 * t + P32 + Q32;
  Temp_P[3][3] = Kalman->P[3][3] + Kalman->Q[3][3];                                       //P33 + Q33;

  Temp_P[4][4] = Kalman->P[5][5] * t * t + Kalman->P[5][4] * t + Kalman->P[4][5] * t + Kalman->P[4][4] + Kalman->Q[4][4];
                                                                                          //P55 * t * t + P54 * t + P45 * t + P44 + Q44;
  Temp_P[4][5] = Kalman->P[5][5] * t + Kalman->P[4][5] + Kalman->Q[4][5];                 //P55 * t + P45 + Q45;
  Temp_P[5][4] = Kalman->P[5][5] * t + Kalman->P[5][4] + Kalman->Q[5][4];                 //P55 * t + P54 + Q54;
  Temp_P[5][5] = Kalman->P[5][5] + Kalman->Q[5][5];                                       //P55 + Q55;

  // memcpy()
  Kalman->P[0][0] = Temp_P[0][0];
  Kalman->P[0][1] = Temp_P[0][1];
  Kalman->P[1][0] = Temp_P[1][0];
  Kalman->P[1][1] = Temp_P[1][1];

  Kalman->P[2][2] = Temp_P[2][2];
  Kalman->P[2][3] = Temp_P[2][3];
  Kalman->P[3][2] = Temp_P[3][2];
  Kalman->P[3][3] = Temp_P[3][3];

  Kalman->P[4][4] = Temp_P[4][4];
  Kalman->P[4][5] = Temp_P[4][5];
  Kalman->P[5][4] = Temp_P[5][4];
  Kalman->P[5][5] = Temp_P[5][5];
  //step3
    //1.求逆
  static float Temp_N[6][6] = {0};
  static float Temp_N_den[3] = {0};
  Temp_N_den[0] = Kalman->R[0][0] * Kalman->R[1][1] + Kalman->P[0][0] * Kalman->R[1][1] - Kalman->R[0][1] * Kalman->R[1][0] - Kalman->P[0][1] * Kalman->R[1][0]
                  - Kalman->P[1][0] * Kalman->R[0][1] + Kalman->P[1][1] * Kalman->R[0][0] + Kalman->P[0][0] * Kalman->P[1][1] - Kalman->P[0][1] * Kalman->P[1][0];
  Temp_N_den[1] = Kalman->R[2][2] * Kalman->R[3][3] + Kalman->P[2][2] * Kalman->R[3][3] - Kalman->R[2][3] * Kalman->R[3][2] - Kalman->P[2][3] * Kalman->R[3][2]
                  - Kalman->P[3][2] * Kalman->R[2][3] + Kalman->P[3][3] * Kalman->R[2][2] + Kalman->P[2][2] * Kalman->P[3][3] - Kalman->P[2][3] * Kalman->P[3][2];
  Temp_N_den[2] = Kalman->R[4][4] * Kalman->R[5][5] + Kalman->P[4][4] * Kalman->R[5][5] - Kalman->R[4][5] * Kalman->R[5][4] - Kalman->P[4][5] * Kalman->R[5][4]
                  - Kalman->P[5][4] * Kalman->R[4][5] + Kalman->P[5][5] * Kalman->R[4][4] + Kalman->P[4][4] * Kalman->P[5][5] - Kalman->P[4][5] * Kalman->P[5][4];
  Temp_N[0][0] = ( Kalman->R[1][1] + Kalman->P[1][1] ) / Temp_N_den[0];
  Temp_N[0][1] = - ( Kalman->R[0][1] + Kalman->P[0][1] ) / Temp_N_den[0];
  Temp_N[1][0] = - ( Kalman->R[1][0] + Kalman->P[1][0] ) / Temp_N_den[0];
  Temp_N[1][1] = ( Kalman->R[0][0] + Kalman->P[0][0] ) / Temp_N_den[0];

  Temp_N[2][2] = ( Kalman->R[3][3] + Kalman->P[3][3] ) / Temp_N_den[1];
  Temp_N[2][3] = - ( Kalman->R[2][3] + Kalman->P[2][3] ) / Temp_N_den[1];
  Temp_N[3][2] = - ( Kalman->R[3][2] + Kalman->P[3][2] ) / Temp_N_den[1];
  Temp_N[3][3] = ( Kalman->R[2][2] + Kalman->P[2][2] ) / Temp_N_den[1];

  Temp_N[4][4] = ( Kalman->R[5][5] + Kalman->P[5][5] ) / Temp_N_den[2];
  Temp_N[4][5] = - ( Kalman->R[4][5] + Kalman->P[4][5] ) / Temp_N_den[2];
  Temp_N[5][4] = - ( Kalman->R[5][4] + Kalman->P[5][4] ) / Temp_N_den[2];
  Temp_N[5][5] = ( Kalman->R[4][4] + Kalman->P[4][4] ) / Temp_N_den[2];
    //2.
  Kalman->K[0][0] = Temp_N[1][0] * Kalman->P[0][1] + Temp_N[0][0] * Kalman->P[0][0];
  Kalman->K[0][1] = Temp_N[1][1] * Kalman->P[0][1] + Temp_N[0][1] * Kalman->P[0][0];
  Kalman->K[1][0] = Temp_N[1][0] * Kalman->P[1][1] + Temp_N[0][0] * Kalman->P[1][0];
  Kalman->K[1][1] = Temp_N[1][1] * Kalman->P[1][1] + Temp_N[0][1] * Kalman->P[1][0];

  Kalman->K[2][2] = Temp_N[3][2] * Kalman->P[2][3] + Temp_N[2][2] * Kalman->P[2][2];
  Kalman->K[2][3] = Temp_N[3][3] * Kalman->P[2][3] + Temp_N[2][3] * Kalman->P[2][2];
  Kalman->K[3][2] = Temp_N[3][2] * Kalman->P[3][3] + Temp_N[2][2] * Kalman->P[3][2];
  Kalman->K[3][3] = Temp_N[3][3] * Kalman->P[3][3] + Temp_N[2][3] * Kalman->P[3][2];

  Kalman->K[4][4] = Temp_N[5][4] * Kalman->P[4][5] + Temp_N[4][4] * Kalman->P[4][4];
  Kalman->K[4][5] = Temp_N[5][5] * Kalman->P[4][5] + Temp_N[4][5] * Kalman->P[4][4];
  Kalman->K[5][4] = Temp_N[5][4] * Kalman->P[5][5] + Temp_N[4][4] * Kalman->P[5][4];
  Kalman->K[5][5] = Temp_N[5][5] * Kalman->P[5][5] + Temp_N[4][5] * Kalman->P[5][4];
  //step4
  Temp_X[0][0] = Kalman->K[0][1] * Z_droll + Kalman->K[0][0] * Z_roll - Kalman->K[0][1] * Kalman->X_hat[1][0] - Kalman->K[0][0] * Kalman->X_hat[0][0] + Kalman->X_hat[0][0];
  Temp_X[1][0] = Kalman->K[1][1] * Z_droll + Kalman->K[1][0] * Z_roll - Kalman->K[1][1] * Kalman->X_hat[1][0] - Kalman->K[1][0] * Kalman->X_hat[0][0] + Kalman->X_hat[1][0];

  Temp_X[2][1] = Kalman->K[2][3] * Z_dpitch + Kalman->K[2][2] * Z_pitch - Kalman->K[2][3] * Kalman->X_hat[3][1] - Kalman->K[2][2] * Kalman->X_hat[2][1] + Kalman->X_hat[2][1];
  Temp_X[3][1] = Kalman->K[3][3] * Z_dpitch + Kalman->K[3][2] * Z_pitch - Kalman->K[3][3] * Kalman->X_hat[3][1] - Kalman->K[3][2] * Kalman->X_hat[2][1] + Kalman->X_hat[3][1];

  Temp_X[4][2] = Kalman->K[4][5] * Z_dyaw + Kalman->K[4][4] * Z_yaw - Kalman->K[4][5] * Kalman->X_hat[5][2] - Kalman->K[4][4] * Kalman->X_hat[4][2] + Kalman->X_hat[4][2];
  Temp_X[5][2] = Kalman->K[5][5] * Z_dyaw + Kalman->K[5][4] * Z_yaw - Kalman->K[5][5] * Kalman->X_hat[5][2] - Kalman->K[5][4] * Kalman->X_hat[4][2] + Kalman->X_hat[5][2];

  // memcpy()
  Kalman->X_hat[0][0] = Temp_X[0][0];
  Kalman->X_hat[1][0] = Temp_X[1][0];
  Kalman->X_hat[2][1] = Temp_X[2][1];
  Kalman->X_hat[3][1] = Temp_X[3][1];
  Kalman->X_hat[4][2] = Temp_X[4][2];
  Kalman->X_hat[5][2] = Temp_X[5][2];
  //step 5
  Temp_P[0][0] = ( - Kalman->K[0][1] * Kalman->P[1][0] ) - Kalman->K[0][0] * Kalman->P[0][0] + Kalman->P[0][0];
  Temp_P[0][1] = ( - Kalman->K[0][1] * Kalman->P[1][1] ) - Kalman->K[0][0] * Kalman->P[0][1] + Kalman->P[0][1];
  Temp_P[1][0] = ( - Kalman->K[1][1] * Kalman->P[1][0] ) - Kalman->K[1][0] * Kalman->P[0][0] + Kalman->P[1][0];
  Temp_P[1][1] = ( - Kalman->K[1][1] * Kalman->P[1][1] ) - Kalman->K[1][0] * Kalman->P[0][1] + Kalman->P[1][1];

  Temp_P[2][2] = ( - Kalman->K[2][3] * Kalman->P[3][2] ) - Kalman->K[2][2] * Kalman->P[2][2] + Kalman->P[2][2];
  Temp_P[2][3] = ( - Kalman->K[2][3] * Kalman->P[3][3] ) - Kalman->K[2][2] * Kalman->P[2][3] + Kalman->P[2][3];
  Temp_P[3][2] = ( - Kalman->K[3][3] * Kalman->P[3][2] ) - Kalman->K[3][2] * Kalman->P[2][2] + Kalman->P[3][2];
  Temp_P[3][3] = ( - Kalman->K[3][3] * Kalman->P[3][3] ) - Kalman->K[3][2] * Kalman->P[2][3] + Kalman->P[3][3];

  Temp_P[4][4] = ( - Kalman->K[4][5] * Kalman->P[5][4] ) - Kalman->K[4][4] * Kalman->P[4][4] + Kalman->P[4][4];
  Temp_P[4][5] = ( - Kalman->K[4][5] * Kalman->P[5][5] ) - Kalman->K[4][4] * Kalman->P[4][5] + Kalman->P[4][5];
  Temp_P[5][4] = ( - Kalman->K[5][5] * Kalman->P[5][4] ) - Kalman->K[5][4] * Kalman->P[4][4] + Kalman->P[5][4];
  Temp_P[5][5] = ( - Kalman->K[5][5] * Kalman->P[5][5] ) - Kalman->K[5][4] * Kalman->P[4][5] + Kalman->P[5][5];
  // memcpy()
  Kalman->P[0][0] = Temp_P[0][0];
  Kalman->P[0][1] = Temp_P[0][1];
  Kalman->P[1][0] = Temp_P[1][0];
  Kalman->P[1][1] = Temp_P[1][1];

  Kalman->P[2][2] = Temp_P[2][2];
  Kalman->P[2][3] = Temp_P[2][3];
  Kalman->P[3][2] = Temp_P[3][2];
  Kalman->P[3][3] = Temp_P[3][3];

  Kalman->P[4][4] = Temp_P[4][4];
  Kalman->P[4][5] = Temp_P[4][5];
  Kalman->P[5][4] = Temp_P[5][4];
  Kalman->P[5][5] = Temp_P[5][5];
}
