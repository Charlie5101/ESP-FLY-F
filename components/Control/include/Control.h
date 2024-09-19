#ifndef CONTROL_H
#define CONTROL_H

#include "esp_attr.h"
#include "PID.h"

typedef struct
{
  struct
  {
    float Kp;
    float Ki;
    float Kd;
    float P_Limit;
    float I_Limit;
    float D_Limit;
    float OUT_Limit;
  }Roll;

  struct
  {
    float Kp;
    float Ki;
    float Kd;
    float P_Limit;
    float I_Limit;
    float D_Limit;
    float OUT_Limit;
  }Pitch;

  struct
  {
    float Kp;
    float Ki;
    float Kd;
    float P_Limit;
    float I_Limit;
    float D_Limit;
    float OUT_Limit;
  }Yaw;
  
}Control_PID_param;

typedef struct
{
  struct
  {
    myPID_Classdef Roll;
    myPID_Classdef Pitch;
    myPID_Classdef Yaw;
    // myPID_Classdef Height;
  }PID;

  struct
  {
    float Roll;
    float Pitch;
    float Yaw;
    // float Height;
  }Normal_Data;

  float Throttle;

  struct
  {
    float Throttle_k;
    float Throttle_b;
    float Roll_k;
    float Roll_b;
    float Pitch_k;
    float Pitch_b;
    float Yaw_k;
    float Yaw_b;

    // float (*Thr_Weight)(void* Control);
  }power_param;

  struct
  {
    float Thr_weight;
    // struct
    // {
    //   float Roll_weight;
    //   float Pitch_weight;
    //   float Yaw_weight;
    // }A,B,C,D;
    float Roll_weight;
    float Pitch_weight;
    float Yaw_weight;
    uint8_t motor_bit;
    float Roll_thr;
    float Pitch_thr;
    float Yaw_thr;
  }distribute_var;

  struct
  {
    uint16_t throttle_A;
    uint16_t throttle_B;
    uint16_t throttle_C;
    uint16_t throttle_D;
  }power_out;
  
  void (*init)(void* Control);
  void (*update)(void* Control, float Throttle,
                                float Roll_Target, float Roll_Current,
                                float Pitch_Target, float Pitch_Current,
                                float Yaw_Target, float Yaw_Current);
  void (*cal)(void* Control);
}Control_Classdef;

void Control_Class_init(Control_Classdef* Control, Control_PID_param param);

#endif
