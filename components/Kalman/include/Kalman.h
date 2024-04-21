#ifndef KALMAN__
#define KALMAN__

#include <math.h>

//low Pass Filter
typedef struct myLPF
{
  float OUT,Last_OUT,Sample,alpha;
}myLPF;

//high Pass Filter
typedef struct myHPF
{
  float OUT,Last_OUT,Sample,Last_Sample,alpha;
}myHPF;

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


// void LPF_init(myLPF *LPF,float alpha);
void LPF_init(myLPF *LPF,float Ts,float fc);
float LPF_cal(myLPF *LPF,float Sample);
// void HPF_init(myHPF *HPF,float alpha);
void HPF_init(myHPF *HPF,float Ts,float fc);
float HPF_cal(myHPF *HPF,float Sample);

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

#endif
