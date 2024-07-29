#ifndef KALMAN__
#define KALMAN__

#include <math.h>

//Kalman Filter
typedef struct myKalman_2
{
  float F[2][2];
  float B[2];
  float Q[2][2];
  float R[2][2];
  float H[2][2];
  float P[2][2];
  float K[2][2];
  float Ut;

  float X_Pred[2];
  float X[2];
}myKalman_2;

void Kalman_2_init(myKalman_2 *Kalman,float F_1,float F_2,float F_3,float F_4,
                                      float B_1,float B_2,
                                      float H_1,float H_2,float H_3,float H_4,
                                      float Q_1,float Q_2,float Q_3,float Q_4,
                                      float R_1,float R_2,float R_3,float R_4,
                                      float X_1,float X_2,
                                      float Ut_);
void Kalman_2_cal(myKalman_2 *Kalman,float F_1,float F_2,float F_3,float F_4,
                                     float B_1,float B_2,
                                     float Ut,
                                     float Z_1,float Z_2);

//Kalman Filter
typedef struct imu_Kalman
{
  float A[6][6];
  float B[6][3];
  float Q[6][6];
  float R[6][6];
  float H[6][6];  //3*6
  float P[6][6];
  float K[6][6];  //6*3

  float Ut[3][3];

  float X_hat[6][3];

  float Z[6][3];  //3*3
}imu_Kalman;

#endif
