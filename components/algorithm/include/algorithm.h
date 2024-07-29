#ifndef ALGORITHM_H__
#define ALGORITHM_H__

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

// void LPF_init(myLPF *LPF,float alpha);
void LPF_init(myLPF *LPF,float Ts,float fc);
float LPF_cal(myLPF *LPF,float Sample);
// void HPF_init(myHPF *HPF,float alpha);
void HPF_init(myHPF *HPF,float Ts,float fc);
float HPF_cal(myHPF *HPF,float Sample);

#endif
