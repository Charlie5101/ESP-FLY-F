#include <stdio.h>
#include "motor.h"
#include "bsp_pwm.h"

#define DSHOT_PROT                DSHOT600

void DSHOT_init(DSHOT_Classdef* DSHOT);
void DSHOT_enable_channel(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_disable_channel(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_esc_unlock(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_esc_rotation_set(DSHOT_Classdef* DSHOT, uint8_t id, uint8_t dir);
void DSHOT_throttle_set(DSHOT_Classdef* DSHOT, uint8_t id, uint16_t throttle);
void DSHOT_enter_set(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_exit_and_save_set(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_esc_rotation_reverse(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_set_all_throttle(DSHOT_Classdef* DSHOT, uint16_t throttle_A, uint16_t throttle_B, uint16_t throttle_C, uint16_t throttle_D);

void motor_init(void);
void servo_init(void);
uint16_t Percent_to_Resolution(float target_duty);
void motor_throttle_set(uint8_t id,float throttle);
void servo_out_set(uint8_t id,float throttle);

void DSHOT_init(DSHOT_Classdef* DSHOT)
{
  for(uint8_t i=0;i<4;i++)
  {
    create_dshot_channel(DSHOT->motor_io[i], DSHOT_PROT, &DSHOT->dshot_channal[i], &DSHOT->dshot_encoder[i]);
  }
  //enable
  for(uint8_t i=0;i<4;i++)
  {
    enable_dshot_channel(DSHOT->dshot_channal[i]);
  }
}

void DSHOT_enable_channel(DSHOT_Classdef* DSHOT, uint8_t id)
{
  enable_dshot_channel(DSHOT->dshot_channal[id-1]);
}

void DSHOT_disable_channel(DSHOT_Classdef* DSHOT, uint8_t id)
{
  disable_dshot_channel(DSHOT->dshot_channal[id-1]);
}

void DSHOT_esc_unlock(DSHOT_Classdef* DSHOT, uint8_t id)
{
  switch(id)
  {
  case 1:
    dshot_send(0, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 2:
    dshot_send(0, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 3:
    dshot_send(0, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 4:
    dshot_send(0, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  default:
    break;
  }
}

void DSHOT_esc_rotation_set(DSHOT_Classdef* DSHOT, uint8_t id, uint8_t dir)
{
  if(dir == 0)
  {
    switch(id)
    {
    case 1:
      dshot_send(7, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    case 2:
      dshot_send(7, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    case 3:
      dshot_send(7, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    case 4:
      dshot_send(7, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    default:
      break;
    }
  }
  else
  {
    switch(id)
    {
    case 1:
      dshot_send(8, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    case 2:
      dshot_send(8, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    case 3:
      dshot_send(8, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    case 4:
      dshot_send(8, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
      break;
    default:
      break;
    }
  }
}

void DSHOT_throttle_set(DSHOT_Classdef* DSHOT, uint8_t id, uint16_t throttle)
{
  if(throttle > 1999)
  {
    throttle = 1999;
  }

  switch(id)
  {
  case 1:
    dshot_send(throttle + 48, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 2:
    dshot_send(throttle + 48, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 3:
    dshot_send(throttle + 48, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 4:
    dshot_send(throttle + 48, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  default:
    break;
  }
}

void DSHOT_enter_set(DSHOT_Classdef* DSHOT, uint8_t id)
{
  switch(id)
  {
  case 1:
    dshot_send(11, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 2:
    dshot_send(11, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 3:
    dshot_send(11, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 4:
    dshot_send(11, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  default:
    break;
  }
}

void DSHOT_exit_and_save_set(DSHOT_Classdef* DSHOT, uint8_t id)
{
  switch(id)
  {
  case 1:
    dshot_send(12, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 2:
    dshot_send(12, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 3:
    dshot_send(12, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 4:
    dshot_send(12, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  default:
    break;
  }
}

void DSHOT_esc_rotation_reverse(DSHOT_Classdef* DSHOT, uint8_t id)
{
  switch(id)
  {
  case 1:
    dshot_send(21, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 2:
    dshot_send(21, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 3:
    dshot_send(21, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  case 4:
    dshot_send(21, false, DSHOT->dshot_channal[id-1], DSHOT->dshot_encoder[id-1]);
    break;
  default:
    break;
  }
}

void DSHOT_set_all_throttle(DSHOT_Classdef* DSHOT, uint16_t throttle_A, uint16_t throttle_B, uint16_t throttle_C, uint16_t throttle_D)
{
  DSHOT_throttle_set(DSHOT, 1, throttle_A);
  DSHOT_throttle_set(DSHOT, 2, throttle_B);
  DSHOT_throttle_set(DSHOT, 3, throttle_C);
  DSHOT_throttle_set(DSHOT, 4, throttle_D);
}

void DSHOT_Class_init(DSHOT_Classdef* DSHOT)
{
  DSHOT->motor_io[0] = M1_IO;
  DSHOT->motor_io[1] = M2_IO;
  DSHOT->motor_io[2] = M3_IO;
  DSHOT->motor_io[3] = M4_IO;
  DSHOT->dshot_channal[0] = NULL;
  DSHOT->dshot_channal[1] = NULL;
  DSHOT->dshot_channal[2] = NULL;
  DSHOT->dshot_channal[3] = NULL;
  DSHOT->dshot_encoder[0] = NULL;
  DSHOT->dshot_encoder[1] = NULL;
  DSHOT->dshot_encoder[2] = NULL;
  DSHOT->dshot_encoder[3] = NULL;

  DSHOT->init = (void (*)(void*))DSHOT_init;
  DSHOT->channel_enable = (void (*)(void*, uint8_t id))DSHOT_enable_channel;
  DSHOT->channel_disable = (void (*)(void*, uint8_t id))DSHOT_disable_channel;
  DSHOT->ESC_unLock = (void (*)(void*, uint8_t id))DSHOT_esc_unlock;
  DSHOT->ESC_Rotation_Set = (void (*)(void*, uint8_t id, uint8_t dir))DSHOT_esc_rotation_set;
  DSHOT->Set_Throttle = (void (*)(void*, uint8_t id, uint16_t throttle))DSHOT_throttle_set;
  DSHOT->Enter_Set = (void (*)(void*, uint8_t id))DSHOT_enter_set;
  DSHOT->Exit_and_Save_Set = (void (*)(void*, uint8_t id))DSHOT_exit_and_save_set;
  DSHOT->ESC_Rotation_Reverse = (void (*)(void*, uint8_t id))DSHOT_esc_rotation_reverse;
  DSHOT->Set_All_Throttle = (void (*)(void*, uint16_t throttle_A, uint16_t throttle_B, uint16_t throttle_C, uint16_t throttle_D))DSHOT_set_all_throttle;

  DSHOT->init(DSHOT);
}

void motor_init()
{
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M1_IO,M1_CHANNEL,0,0);
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M2_IO,M2_CHANNEL,0,0);
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M3_IO,M3_CHANNEL,0,0);
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M4_IO,M4_CHANNEL,0,0);
}

void servo_init()
{
  pwm_init(SERVO_TIMER,SERVO_RESOLUTION,SERVO_REFRESH_RATE,S1_IO,S1_CHANNEL,0,0);
  pwm_init(SERVO_TIMER,SERVO_RESOLUTION,SERVO_REFRESH_RATE,S2_IO,S2_CHANNEL,0,0);
  pwm_init(SERVO_TIMER,SERVO_RESOLUTION,SERVO_REFRESH_RATE,S3_IO,S3_CHANNEL,0,0);
  pwm_init(SERVO_TIMER,SERVO_RESOLUTION,SERVO_REFRESH_RATE,S4_IO,S4_CHANNEL,0,0);
}

uint16_t Percent_to_Resolution(float target_duty)
{
  return (uint16_t)( (target_duty/100.0) * 16383);
}

void motor_throttle_set(uint8_t id,float throttle)
{
  switch(id)
  {
  case 1:
    pwm_set_duty(M1_CHANNEL,Percent_to_Resolution(throttle));
    break;
  case 2:
    pwm_set_duty(M2_CHANNEL,Percent_to_Resolution(throttle));
    break;
  case 3:
    pwm_set_duty(M3_CHANNEL,Percent_to_Resolution(throttle));
    break;
  case 4:
    pwm_set_duty(M4_CHANNEL,Percent_to_Resolution(throttle));
    break;
  default:
    break;
  }
}

void servo_out_set(uint8_t id,float throttle)
{
  switch(id)
  {
  case 1:
    pwm_set_duty(S1_CHANNEL,Percent_to_Resolution(throttle));
    break;
  case 2:
    pwm_set_duty(S2_CHANNEL,Percent_to_Resolution(throttle));
    break;
  case 3:
    pwm_set_duty(S3_CHANNEL,Percent_to_Resolution(throttle));
    break;
  case 4:
    pwm_set_duty(S4_CHANNEL,Percent_to_Resolution(throttle));
    break;
  default:
    break;
  }
}

void Motor_Class_init(Motor_Classdef* Motor)
{
  Motor->init = (void (*)(void))motor_init;
  Motor->Per_to_Res = (uint16_t (*)(float target_duty))Percent_to_Resolution;
  Motor->Set_Throttle = (void (*)(uint8_t id, float throttle))motor_throttle_set;

  Motor->init();
}

void Servo_Class_init(Servo_Classdef* Servo)
{
  Servo->init = (void (*)(void))servo_init;
  Servo->Per_to_Res = (uint16_t (*)(float target_duty))Percent_to_Resolution;
  Servo->Set_Out = (void (*)(uint8_t id, float throttle))servo_out_set;

  Servo->init();
}
