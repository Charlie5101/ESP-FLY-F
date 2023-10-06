#include <stdio.h>
#include "Kalman.h"

#define pi 3.1415926f
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


