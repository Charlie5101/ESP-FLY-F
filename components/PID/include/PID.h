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

void PID_param_init(myPID *PID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit);
void PID_param_switch(myPID *PID,float Kp,float Ki,float Kd,float P_Limit,float I_Limit,float D_Limit);
float PID_cal(myPID *PID,float Current,float Target);

#endif
