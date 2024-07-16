#ifndef myPID__
#define myPID__

#include <math.h>

typedef struct myPID
{
  // const float Kp,Ki,Kd,P_Limit,I_Limit,D_Limit;
  float Kp,Ki,Kd,P_Limit,I_Limit,D_Limit;

  float Target,Current,Error,Last_Error;
  float P_OUT,I_OUT,D_OUT,OUT;

}myPID;

typedef struct intmyPID
{
  // const uint32_t Kp,Ki,Kd,P_Limit,I_Limit,D_Limit;
  int32_t Kp,Ki,Kd,P_Limit,I_Limit,D_Limit;

  int32_t Target,Current,Error,Last_Error;
  int32_t P_OUT,I_OUT,D_OUT,OUT;

}intmyPID;

typedef struct
{
  struct param
  {
    float Kp,Ki,Kd,P_Limit,I_Limit,D_Limit,OUT_Limit;
  }param;
  struct data
  {
    float Target,Current,Error,Last_Error;
  }data;

  struct T_out
  {
    float P,I,D;
  }T_out;

  float OUT;

  void (*init)(void *myPID);
  void (*switch_param)(void *myPID, float Kp, float Ki, float Kd, float P_Limit, float I_Limit, float D_Limit);
  void (*update)(void *myPID, float Target, float Current);
  void (*cal)(void* myPID);

}myPID_Classdef;

void myPID_Class_init(myPID_Classdef* myPID, float Kp, float Ki, float Kd, float P_Limit, float I_Limit, float D_Limit, float OUT_Limit);
/*
void myPID_init(myPID_Classdef* myPID);
void myPID_param_switch(myPID_Classdef* myPID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit);
void myPID_update(myPID_Classdef *myPID, float Target, float Current);
void myPID_cal(myPID_Classdef* myPID);
*/

void PID_param_init(myPID *PID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit);
void PID_param_switch(myPID *PID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit);
float PID_cal(myPID *PID,float Current,float Target);

void intPID_param_init(intmyPID *PID,int32_t Kp,int32_t Ki,int32_t Kd,int32_t P_Limit,int32_t I_Limit,int32_t D_Limit);
void intPID_param_switch(intmyPID *PID,int32_t Kp,int32_t Ki,int32_t Kd,int32_t P_Limit,int32_t I_Limit,int32_t D_Limit);
int32_t intPID_cal(intmyPID *PID,int32_t Current,int32_t Target);

#endif
