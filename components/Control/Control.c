#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "Control.h"

#define MIN_OUT           199
#define MAX_OUT           1999
#define MIN_THR_WEIGHT    0.8
#define MAX_THR_WEIGHT    1.2

#define FIND_MAX_IN4(a, b, c, d)                          \
({                                                        \
  float max = a;                                          \
  if(max - b < 0)                                         \
  {                                                       \
    max = b;                                              \
  }                                                       \
  if(max - c < 0)                                         \
  {                                                       \
    max = c;                                              \
  }                                                       \
  if(max - d < 0)                                         \
  {                                                       \
    max = d;                                              \
  }                                                       \
  max;                                                    \
})

#define FIND_MIN_IN4(a, b, c, d)                          \
({                                                        \
  float min = a;                                          \
  if(min - b > 0)                                         \
  {                                                       \
    min = b;                                              \
  }                                                       \
  if(min - c < 0)                                         \
  {                                                       \
    min = c;                                              \
  }                                                       \
  if(min - d < 0)                                         \
  {                                                       \
    min = d;                                              \
  }                                                       \
  min;                                                    \
})

void Control_init(Control_Classdef* Control);
void Control_update(Control_Classdef* Control, float Throttle,
                                               float Roll_Target, float Roll_Current,
                                               float Pitch_Target, float Pitch_Current,
                                               float Yaw_Target, float Yaw_Current);
void Control_cal(Control_Classdef* Control);
float Get_Thr_Weight(Control_Classdef* Control);
float* Find_MAX_IN4(float* a, float* b, float* c, float* d);
float* Find_MIN_IN4(float* a, float* b, float* c, float* d);
float* Find_ABSF_MAX_IN2(float*a, float* b);

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

  Control->power_param.Throttle_k = 2000;
  Control->power_param.Throttle_b = -160.0;
  Control->power_param.Roll_k = 1000.0;
  Control->power_param.Roll_b = 0.0;
  Control->power_param.Pitch_k = 1000.0;
  Control->power_param.Pitch_b = 0.0;
  Control->power_param.Yaw_k = 1000.0;
  Control->power_param.Yaw_b = 0.0;

  Control->distribute_var.Thr_weight = 1.0;
  Control->distribute_var.Roll_weight = 0.0;
  Control->distribute_var.Pitch_weight = 0.0;
  Control->distribute_var.Yaw_weight = 0.0;
  Control->distribute_var.motor_bit = 0;
  Control->distribute_var.Roll_thr = 0.0;
  Control->distribute_var.Pitch_thr = 0.0;
  Control->distribute_var.Yaw_thr = 0.0;

  Control->init = (void (*)(void*))Control_init;
  Control->update = (void (*)(void*, float Thtottle,
                                     float Roll_Target, float Roll_Current,
                                     float Pitch_Target, float Pitch_Current,
                                     float Yaw_Target, float Yaw_Current))Control_update;
  Control->cal = (void (*)(void*))Control_cal;
  // Control->power_param.Thr_Weight = (float (*)(void*))Get_Thr_Weight;

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

  static float temp_throttle = 0.0;

  Control->PID.Roll.cal(&Control->PID.Roll);
  Control->PID.Pitch.cal(&Control->PID.Pitch);
  Control->PID.Yaw.cal(&Control->PID.Yaw);
  Control->Normal_Data.Roll = Control->PID.Roll.OUT / Control->PID.Roll.param.OUT_Limit;
  Control->Normal_Data.Pitch = Control->PID.Pitch.OUT / Control->PID.Pitch.param.OUT_Limit;
  Control->Normal_Data.Yaw = Control->PID.Yaw.OUT / Control->PID.Yaw.param.OUT_Limit;

  temp_throttle = (Control->power_param.Throttle_k * Control->Throttle + Control->power_param.Throttle_b);
  Control->distribute_var.Roll_thr = (Control->power_param.Roll_k * Control->Normal_Data.Roll + Control->power_param.Roll_b);
  Control->distribute_var.Pitch_thr = (Control->power_param.Pitch_k * Control->Normal_Data.Pitch + Control->power_param.Pitch_b);
  Control->distribute_var.Yaw_thr = (Control->power_param.Yaw_k * Control->Normal_Data.Yaw + Control->power_param.Yaw_b);

  if(temp_throttle <= MIN_OUT)
  {
    temp_throttle_A = 0;
    temp_throttle_B = 0;
    temp_throttle_C = 0;
    temp_throttle_D = 0;
  }
  else
  {
    temp_throttle_A = temp_throttle
                      - Control->distribute_var.Roll_thr
                      + Control->distribute_var.Pitch_thr
                      + Control->distribute_var.Yaw_thr;
    temp_throttle_B = temp_throttle
                      - Control->distribute_var.Roll_thr
                      - Control->distribute_var.Pitch_thr
                      - Control->distribute_var.Yaw_thr;
    temp_throttle_C = temp_throttle
                      + Control->distribute_var.Roll_thr
                      + Control->distribute_var.Pitch_thr
                      - Control->distribute_var.Yaw_thr;
    temp_throttle_D = temp_throttle
                      + Control->distribute_var.Roll_thr
                      - Control->distribute_var.Pitch_thr
                      + Control->distribute_var.Yaw_thr;

    if(temp_throttle_A <= MIN_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b00000001;
    }
    else if(temp_throttle_A >= MAX_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b00010000;
    }
    if(temp_throttle_B <= MIN_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b00000010;
    }
    else if(temp_throttle_B >= MAX_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b00100000;
    }
    if(temp_throttle_C <= MIN_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b00000100;
    }
    else if(temp_throttle_C >= MAX_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b01000000;
    }
    if(temp_throttle_D <= MIN_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b00001000;
    }
    else if(temp_throttle_D >= MAX_OUT)
    {
      Control->distribute_var.motor_bit = Control->distribute_var.motor_bit | 0b10000000;
    }
    // ESP_LOGI("Control", "%x:", Control->distribute_var.motor_bit);
  }

  if((Control->distribute_var.motor_bit & 0b11110000) != 0 && (Control->distribute_var.motor_bit & 0b00001111) == 0)
  {
    static float* MAX_motor = NULL;
    MAX_motor = Find_MAX_IN4(&temp_throttle_A, &temp_throttle_B, &temp_throttle_C, &temp_throttle_D);
    Control->distribute_var.Thr_weight = (MAX_OUT - *MAX_motor + temp_throttle) / temp_throttle;
    if(Control->distribute_var.Thr_weight >= MIN_THR_WEIGHT)
    {
      //OUT
      temp_throttle_A = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
      temp_throttle_B = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_C = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_D = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
    }
    else
    {
      Control->distribute_var.Thr_weight = MIN_THR_WEIGHT;
      Control->distribute_var.Roll_weight = Control->distribute_var.Roll_thr / (*MAX_motor - temp_throttle);
      Control->distribute_var.Pitch_weight = Control->distribute_var.Pitch_thr / (*MAX_motor - temp_throttle);
      Control->distribute_var.Yaw_weight = Control->distribute_var.Yaw_thr / (*MAX_motor - temp_throttle);

      Control->distribute_var.Roll_thr = Control->distribute_var.Roll_weight * (MAX_OUT - Control->distribute_var.Thr_weight * temp_throttle);
      Control->distribute_var.Pitch_thr = Control->distribute_var.Pitch_weight * (MAX_OUT - Control->distribute_var.Thr_weight * temp_throttle);
      Control->distribute_var.Yaw_thr = Control->distribute_var.Yaw_weight * (MAX_OUT - Control->distribute_var.Thr_weight * temp_throttle);

      temp_throttle_A = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
      temp_throttle_B = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_C = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_D = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
    }
    Control->distribute_var.motor_bit = 0;
  }
  else if((Control->distribute_var.motor_bit & 0b00001111) != 0 && (Control->distribute_var.motor_bit & 0b11110000) == 0)
  {
    static float* MIN_motor = NULL;
    MIN_motor = Find_MIN_IN4(&temp_throttle_A, &temp_throttle_B, &temp_throttle_C, &temp_throttle_D);
    Control->distribute_var.Thr_weight = (MIN_OUT - *MIN_motor + temp_throttle) / temp_throttle;
    if(Control->distribute_var.Thr_weight <= MAX_THR_WEIGHT)
    {
      //OUT
      temp_throttle_A = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
      temp_throttle_B = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_C = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_D = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
    }
    else
    {
      Control->distribute_var.Thr_weight = MAX_THR_WEIGHT;
      Control->distribute_var.Roll_weight = Control->distribute_var.Roll_thr / (*MIN_motor - temp_throttle);
      Control->distribute_var.Pitch_weight = Control->distribute_var.Pitch_thr / (*MIN_motor - temp_throttle);
      Control->distribute_var.Yaw_weight = Control->distribute_var.Yaw_thr / (*MIN_motor - temp_throttle);

      Control->distribute_var.Roll_thr = Control->distribute_var.Roll_weight * (MIN_OUT - Control->distribute_var.Thr_weight * temp_throttle);
      Control->distribute_var.Pitch_thr = Control->distribute_var.Pitch_weight * (MIN_OUT - Control->distribute_var.Thr_weight * temp_throttle);
      Control->distribute_var.Yaw_thr = Control->distribute_var.Yaw_weight * (MIN_OUT - Control->distribute_var.Thr_weight * temp_throttle);

      temp_throttle_A = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
      temp_throttle_B = Control->distribute_var.Thr_weight * temp_throttle
                        - Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_C = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        + Control->distribute_var.Pitch_thr
                        - Control->distribute_var.Yaw_thr;
      temp_throttle_D = Control->distribute_var.Thr_weight * temp_throttle
                        + Control->distribute_var.Roll_thr
                        - Control->distribute_var.Pitch_thr
                        + Control->distribute_var.Yaw_thr;
    }
    Control->distribute_var.motor_bit = 0;
  }
  else if((Control->distribute_var.motor_bit & 0b00001111) != 0 && (Control->distribute_var.motor_bit & 0b11110000) != 0)
  {
    //NEED CHANGE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Control->distribute_var.Thr_weight = 1.0;
    temp_throttle_A -= temp_throttle;
    temp_throttle_B -= temp_throttle;
    temp_throttle_C -= temp_throttle;
    temp_throttle_D -= temp_throttle;
    static float* MAX_motor = NULL;
    MAX_motor = Find_MAX_IN4(&temp_throttle_A, &temp_throttle_B, &temp_throttle_C, &temp_throttle_D);
    static float* MIN_motor = NULL;
    MIN_motor = Find_MIN_IN4(&temp_throttle_A, &temp_throttle_B, &temp_throttle_C, &temp_throttle_D);
    static float* MAX = NULL;
    MAX = Find_ABSF_MAX_IN2(MAX_motor, MIN_motor);
    // Control->distribute_var.Roll_weight = Control->distribute_var.Roll_thr / (*MAX);
    // Control->distribute_var.Pitch_weight = Control->distribute_var.Pitch_thr / (*MAX);
    // Control->distribute_var.Yaw_weight = Control->distribute_var.Yaw_thr / (*MAX);
    if(*MAX >= MAX_OUT)
    {
      Control->distribute_var.RPY_weight = (MAX_OUT - temp_throttle) / (*MAX);
    }
    else if(*MAX < MIN_OUT)
    {
      Control->distribute_var.RPY_weight = (MIN_OUT - temp_throttle) / (*MAX);
    }
    temp_throttle_A = temp_throttle + Control->distribute_var.RPY_weight * (temp_throttle_A);
    temp_throttle_B = temp_throttle + Control->distribute_var.RPY_weight * (temp_throttle_B);
    temp_throttle_C = temp_throttle + Control->distribute_var.RPY_weight * (temp_throttle_C);
    temp_throttle_D = temp_throttle + Control->distribute_var.RPY_weight * (temp_throttle_D);
    ESP_LOGI("Control", "weight: %f MAX: %f", Control->distribute_var.RPY_weight, *MAX);

    Control->distribute_var.motor_bit = 0;
  }
  else{}

  // if(temp_throttle_A <= 0)
  // {
  //   temp_throttle_A = 0;
  // }
  // else if(temp_throttle_A >= 1999)
  // {
  //   temp_throttle_A = 1999;
  // }
  // if(temp_throttle_B <= 0)
  // {
  //   temp_throttle_B = 0;
  // }
  // else if(temp_throttle_B >= 1999)
  // {
  //   temp_throttle_B = 1999;
  // }
  // if(temp_throttle_C <= 0)
  // {
  //   temp_throttle_C = 0;
  // }
  // else if(temp_throttle_C >= 1999)
  // {
  //   temp_throttle_C = 1999;
  // }
  // if(temp_throttle_D <= 0)
  // {
  //   temp_throttle_D = 0;
  // }
  // else if(temp_throttle_D >= 1999)
  // {
  //   temp_throttle_D = 1999;
  // }

  Control->power_out.throttle_A = (uint16_t)temp_throttle_A;
  Control->power_out.throttle_B = (uint16_t)temp_throttle_B;
  Control->power_out.throttle_C = (uint16_t)temp_throttle_C;
  Control->power_out.throttle_D = (uint16_t)temp_throttle_D;
}

float Get_Thr_Weight(Control_Classdef* Control)
{
  if(Control->Throttle >= 0 && Control->Throttle < 0.75)
  {
    return 0.8;
  }
  else if(Control->Throttle >= 0.75)
  {
    return 1;
  }
  return 0;
}

float* Find_MAX_IN4(float* a, float* b, float* c, float* d)
{
  float* max = a;
  if(*max - *b < 0)
  {
    max = b;
  }
  if(*max - *c < 0)
  {
    max = c;
  }
  if(*max - *d < 0)
  {
    max = d;
  }
  return max;
}

float* Find_MIN_IN4(float* a, float* b, float* c, float* d)
{
  float* min = a;
  if(*min - *b > 0)
  {
    min = b;
  }
  if(*min - *c > 0)
  {
    min = c;
  }
  if(*min - *d > 0)
  {
    min = d;
  }
  return min;
}

float* Find_ABSF_MAX_IN2(float*a, float* b)
{
  float* max = a;
  if(fabsf(*max) - fabsf(*b) < 0)
  {
    max = b;
  }
  return max;
}

