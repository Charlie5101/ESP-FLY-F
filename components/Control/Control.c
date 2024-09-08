#include <stdio.h>
#include "esp_log.h"
#include "Control.h"

void Control_init(Control_Classdef* Control);
void Control_update(Control_Classdef* Control, float Throttle,
                                               float Roll_Target, float Roll_Current,
                                               float Pitch_Target, float Pitch_Current,
                                               float Yaw_Target, float Yaw_Current);
void Control_cal(Control_Classdef* Control);

void Control_Class_init(Control_Classdef* Control, Control_PID_param param)
{
  Control->Normal_Data.Roll = 0.0;
  Control->Normal_Data.Pitch = 0.0;
  Control->Normal_Data.Yaw = 0.0;
  Control->Throttle = 0.0;
  Control->power_out.throttle_A = 0;
  Control->power_out.throttle_B = 0;
  Control->power_out.throttle_C = 0;
  Control->power_out.throttle_D = 0;

  Control->power_param.Throttle_k = 1800;
  Control->power_param.Throttle_b = -160.0;
  Control->power_param.Roll_k = 400.0;
  Control->power_param.Roll_b = 0.0;
  Control->power_param.Pitch_k = 400.0;
  Control->power_param.Pitch_b = 0.0;
  Control->power_param.Yaw_k = 400.0;
  Control->power_param.Yaw_b = 0.0;

  Control->init = (void (*)(void*))Control_init;
  Control->update = (void (*)(void*, float Thtottle,
                                     float Roll_Target, float Roll_Current,
                                     float Pitch_Target, float Pitch_Current,
                                     float Yaw_Target, float Yaw_Current))Control_update;
  Control->cal = (void (*)(void*))Control_cal;

  myPID_Class_init(&Control->PID.Roll, param.Roll.Kp, param.Roll.Ki, param.Roll.Kd,
                    param.Roll.P_Limit, param.Roll.I_Limit, param.Roll.D_Limit, param.Roll.OUT_Limit);
  myPID_Class_init(&Control->PID.Pitch, param.Pitch.Kp, param.Pitch.Ki, param.Pitch.Kd,
                    param.Pitch.P_Limit, param.Pitch.I_Limit, param.Pitch.D_Limit, param.Pitch.OUT_Limit);
  myPID_Class_init(&Control->PID.Yaw, param.Yaw.Kp, param.Yaw.Ki, param.Yaw.Kd,
                    param.Yaw.P_Limit, param.Yaw.I_Limit, param.Yaw.D_Limit, param.Yaw.OUT_Limit);

  Control->init(Control);
}

void IRAM_ATTR Control_init(Control_Classdef* Control)
{

}

void IRAM_ATTR Control_update(Control_Classdef* Control, float Throttle,
                                                         float Roll_Target, float Roll_Current,
                                                         float Pitch_Target, float Pitch_Current,
                                                         float Yaw_Target, float Yaw_Current)
{
  Control->Throttle = Throttle;
  Control->PID.Roll.update(&Control->PID.Roll, Roll_Target, Roll_Current);
  Control->PID.Pitch.update(&Control->PID.Pitch, Pitch_Target, Pitch_Current);
  Control->PID.Yaw.update(&Control->PID.Yaw, Yaw_Target, Yaw_Current);
}

void IRAM_ATTR Control_cal(Control_Classdef* Control)
{
  static float temp_throttle_A = 0.0;
  static float temp_throttle_B = 0.0;
  static float temp_throttle_C = 0.0;
  static float temp_throttle_D = 0.0;
  Control->PID.Roll.cal(&Control->PID.Roll);
  Control->PID.Pitch.cal(&Control->PID.Pitch);
  Control->PID.Yaw.cal(&Control->PID.Yaw);
  Control->Normal_Data.Roll = Control->PID.Roll.OUT / Control->PID.Roll.param.OUT_Limit;
  Control->Normal_Data.Pitch = Control->PID.Pitch.OUT / Control->PID.Pitch.param.OUT_Limit;
  Control->Normal_Data.Yaw = Control->PID.Yaw.OUT / Control->PID.Yaw.param.OUT_Limit;

  temp_throttle_A = (Control->power_param.Throttle_k * Control->Throttle + Control->power_param.Throttle_b)
                    - (Control->power_param.Roll_k * Control->Normal_Data.Roll + Control->power_param.Roll_b)
                    + (Control->power_param.Pitch_k * Control->Normal_Data.Pitch + Control->power_param.Pitch_b)
                    + (Control->power_param.Yaw_k * Control->Normal_Data.Yaw + Control->power_param.Yaw_b);
  temp_throttle_B = (Control->power_param.Throttle_k * Control->Throttle + Control->power_param.Throttle_b)
                    - (Control->power_param.Roll_k * Control->Normal_Data.Roll + Control->power_param.Roll_b)
                    - (Control->power_param.Pitch_k * Control->Normal_Data.Pitch + Control->power_param.Pitch_b)
                    - (Control->power_param.Yaw_k * Control->Normal_Data.Yaw + Control->power_param.Yaw_b);
  temp_throttle_C = (Control->power_param.Throttle_k * Control->Throttle + Control->power_param.Throttle_b)
                    + (Control->power_param.Roll_k * Control->Normal_Data.Roll + Control->power_param.Roll_b)
                    + (Control->power_param.Pitch_k * Control->Normal_Data.Pitch + Control->power_param.Pitch_b)
                    - (Control->power_param.Yaw_k * Control->Normal_Data.Yaw + Control->power_param.Yaw_b);
  temp_throttle_D = (Control->power_param.Throttle_k * Control->Throttle + Control->power_param.Throttle_b)
                    + (Control->power_param.Roll_k * Control->Normal_Data.Roll + Control->power_param.Roll_b)
                    - (Control->power_param.Pitch_k * Control->Normal_Data.Pitch + Control->power_param.Pitch_b)
                    + (Control->power_param.Yaw_k * Control->Normal_Data.Yaw + Control->power_param.Yaw_b);
  // temp_throttle_A = Control->Throttle * 2000 - 170;
  // temp_throttle_B = Control->Throttle * 2000 - 170;
  // temp_throttle_C = Control->Throttle * 2000 - 170;
  // temp_throttle_D = Control->Throttle * 2000 - 170;

  if(temp_throttle_A <= 0)
  {
    temp_throttle_A = 0;
  }
  else if(temp_throttle_A >= 1999)
  {
    temp_throttle_A = 1999;
  }
  if(temp_throttle_B <= 0)
  {
    temp_throttle_B = 0;
  }
  else if(temp_throttle_B >= 1999)
  {
    temp_throttle_B = 1999;
  }
  if(temp_throttle_C <= 0)
  {
    temp_throttle_C = 0;
  }
  else if(temp_throttle_C >= 1999)
  {
    temp_throttle_C = 1999;
  }
  if(temp_throttle_D <= 0)
  {
    temp_throttle_D = 0;
  }
  else if(temp_throttle_D >= 1999)
  {
    temp_throttle_D = 1999;
  }

  Control->power_out.throttle_A = (uint16_t)temp_throttle_A;
  Control->power_out.throttle_B = (uint16_t)temp_throttle_B;
  Control->power_out.throttle_C = (uint16_t)temp_throttle_C;
  Control->power_out.throttle_D = (uint16_t)temp_throttle_D;
}
