#include <stdio.h>
#include "PID.h"

void PID_param_init(myPID *PID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit)
{
  PID->Kp = Kp;
  PID->Ki = Ki;
  PID->Kd = Kd;
  PID->P_Limit = P_Limit;
  PID->I_Limit = I_Limit;
  PID->D_Limit = D_Limit;

  PID->Current = PID->Target = PID->Error = PID->Last_Error = 0.0;
  PID->P_OUT = PID->I_OUT = PID->D_OUT = 0.0;
}

void PID_param_switch(myPID *PID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit)
{
  PID->Kp = Kp;
  PID->Ki = Ki;
  PID->Kd = Kd;
  PID->P_Limit = P_Limit;
  PID->I_Limit = I_Limit;
  PID->D_Limit = D_Limit;
}

float PID_cal(myPID *PID,float Current,float Target)
{
  //Parallel PID
  PID->Current = Current;
  if(PID->Target != Target)
  {
    PID->I_OUT = 0.0;
    PID->Last_Error = 0.0;
  }
  PID->Target = Target;
  PID->Error = PID->Target - PID->Current;
  PID->P_OUT = PID->Kp * PID->Error;
  PID->I_OUT = PID->Ki * PID->Error + PID->I_OUT;
  PID->D_OUT = PID->Kd * ( PID->Error - PID->Last_Error );
  PID->Last_Error = PID->Error;
  //Limit
  if(fabsf(PID->P_OUT) > PID->P_Limit)
    PID->P_OUT = PID->P_Limit * (PID->P_OUT / fabsf(PID->P_OUT));
  if(fabsf(PID->I_OUT) > PID->I_Limit)
    PID->I_OUT = PID->I_Limit * (PID->I_OUT / fabsf(PID->I_OUT));
  if(fabsf(PID->D_OUT) > PID->D_Limit)
    PID->D_OUT = PID->D_Limit * (PID->D_OUT / fabsf(PID->D_OUT));

  PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;

  return PID->OUT;
}
