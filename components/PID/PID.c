#include <stdio.h>
#include "esp_attr.h"
#include "PID.h"

void myPID_init(myPID_Classdef* myPID);
void myPID_param_switch(myPID_Classdef* myPID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit);
void myPID_update(myPID_Classdef *myPID, float Target, float Current);
void myPID_cal(myPID_Classdef* myPID);

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

  PID->Current = PID->Target = PID->Error = PID->Last_Error = 0.0;
  PID->P_OUT = PID->I_OUT = PID->D_OUT = 0.0;
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

void intPID_param_init(intmyPID *PID,int32_t Kp,int32_t Ki,int32_t Kd,int32_t P_Limit,int32_t I_Limit,int32_t D_Limit)
{
  PID->Kp = Kp;
  PID->Ki = Ki;
  PID->Kd = Kd;
  PID->P_Limit = P_Limit;
  PID->I_Limit = I_Limit;
  PID->D_Limit = D_Limit;

  PID->Current = PID->Target = PID->Error = PID->Last_Error = 0;
  PID->P_OUT = PID->I_OUT = PID->D_OUT = 0;
}

void intPID_param_switch(intmyPID *PID,int32_t Kp,int32_t Ki,int32_t Kd,int32_t P_Limit,int32_t I_Limit,int32_t D_Limit)
{
  PID->Kp = Kp;
  PID->Ki = Ki;
  PID->Kd = Kd;
  PID->P_Limit = P_Limit;
  PID->I_Limit = I_Limit;
  PID->D_Limit = D_Limit;

  PID->Current = PID->Target = PID->Error = PID->Last_Error = 0;
  PID->P_OUT = PID->I_OUT = PID->D_OUT = 0;
}

int32_t intPID_cal(intmyPID *PID,int32_t Current,int32_t Target)
{
  //Parallel PID
  PID->Current = Current;
  if(PID->Target != Target)
  {
    PID->I_OUT = 0;
    PID->Last_Error = 0;
  }
  PID->Target = Target;
  PID->Error = PID->Target - PID->Current;
  PID->P_OUT = PID->Kp * PID->Error;
  PID->I_OUT = PID->Ki * PID->Error + PID->I_OUT;
  PID->D_OUT = PID->Kd * ( PID->Error - PID->Last_Error );
  PID->Last_Error = PID->Error;
  //Limit
  if(abs(PID->P_OUT) > PID->P_Limit)
    PID->P_OUT = PID->P_Limit * (PID->P_OUT / abs(PID->P_OUT));
  if(abs(PID->I_OUT) > PID->I_Limit)
    PID->I_OUT = PID->I_Limit * (PID->I_OUT / abs(PID->I_OUT));
  if(abs(PID->D_OUT) > PID->D_Limit)
    PID->D_OUT = PID->D_Limit * (PID->D_OUT / abs(PID->D_OUT));

  PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;

  return PID->OUT;
}

void myPID_Class_init(myPID_Classdef* myPID, float Kp, float Ki, float Kd, float P_Limit, float I_Limit, float D_Limit, float OUT_Limit)
{
  myPID->T_out.P = myPID->T_out.I = myPID->T_out.D = 0.0f;
  myPID->OUT = 0.0f;
  myPID->data.Target = myPID->data.Current = myPID->data.Error = myPID->data.Last_Error = 0.0f;
  myPID->param.Kp = Kp;
  myPID->param.Ki = Ki;
  myPID->param.Kd = Kd;
  myPID->param.P_Limit = P_Limit;
  myPID->param.I_Limit = I_Limit;
  myPID->param.D_Limit = D_Limit;
  myPID->param.OUT_Limit = OUT_Limit;

  myPID->init = (void (*)(void*))myPID_init;
  myPID->switch_param = (void (*)(void*, float Kp, float Ki, float Kd, float P_Limit, float I_Limit, float D_Limit))myPID_param_switch;
  myPID->update = (void (*)(void*, float Target, float Current))myPID_update;
  myPID->cal = (void (*)(void*))myPID_cal;

  myPID->init(myPID);
}

void myPID_init(myPID_Classdef* myPID)
{

}

void IRAM_ATTR myPID_param_switch(myPID_Classdef* myPID, float Kp, float Ki, float Kd, float P_Limit, float I_Limit, float D_Limit)
{
  myPID->param.Kp = Kp;
  myPID->param.Ki = Ki;
  myPID->param.Kd = Kd;
  myPID->param.P_Limit = P_Limit;
  myPID->param.I_Limit = I_Limit;
  myPID->param.D_Limit = D_Limit;

  // myPID->out.P = myPID->out.I = myPID->out.D = 0.0f;
  // myPID->OUT = 0.0f;
  // myPID->data.Target = myPID->data.Current = myPID->data.Error = myPID->data.Last_Error = 0.0f;
}

void IRAM_ATTR myPID_update(myPID_Classdef *myPID, float Target, float Current)
{
  myPID->data.Current = Current;
  myPID->data.Target = Target;
}

void IRAM_ATTR myPID_cal(myPID_Classdef* myPID)
{
  //Parallel PID
  myPID->data.Error = myPID->data.Target - myPID->data.Current;
  myPID->T_out.P = myPID->param.Kp * myPID->data.Error;
  myPID->T_out.I = myPID->param.Ki * myPID->data.Error + myPID->T_out.I;
  myPID->T_out.D = myPID->param.Kd * ( myPID->data.Error - myPID->data.Last_Error );
  myPID->data.Last_Error = myPID->data.Error;
  //Limit
  if(fabsf(myPID->T_out.P) > myPID->param.P_Limit)
    myPID->T_out.P = myPID->param.P_Limit * (myPID->T_out.P / fabsf(myPID->T_out.P));
  if(fabsf(myPID->T_out.I) > myPID->param.I_Limit)
    myPID->T_out.I = myPID->param.I_Limit * (myPID->T_out.I / fabsf(myPID->T_out.I));
  if(fabsf(myPID->T_out.D) > myPID->param.D_Limit)
    myPID->T_out.D = myPID->param.D_Limit * (myPID->T_out.D / fabsf(myPID->T_out.D));

  myPID->OUT = myPID->T_out.P + myPID->T_out.I + myPID->T_out.D;
  if(fabsf(myPID->OUT) > myPID->param.OUT_Limit)
    myPID->OUT = myPID->param.OUT_Limit * (myPID->OUT / fabsf(myPID->OUT));
}

