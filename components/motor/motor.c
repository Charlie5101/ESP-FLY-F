#include <stdio.h>
#include "motor.h"
#include "DSHOT.h"
#include "bsp_pwm.h"

#define DSHOT_PROT                DSHOT600

int8_t motor_io[4] = {M1_IO, M2_IO, M3_IO, M4_IO};
DSHOT_Channel_handle_t dshot_channal[4] = {NULL};
DSHOT_encoder_handle_t dshot_encoder[4] = {NULL};

void DSHOT_init()
{
  for(uint8_t i=0;i<4;i++)
  {
    create_dshot_channel(motor_io[i], DSHOT_PROT, &dshot_channal[i], &dshot_encoder[i]);
  }
  //enable
  for(uint8_t i=0;i<4;i++)
  {
    enable_dshot_channel(dshot_channal[i]);
  }
}

void DSHOT_enable_channel(uint8_t id)
{
  enable_dshot_channel(dshot_channal[id-1]);
}

void DSHOT_disable_channel(uint8_t id)
{
  disable_dshot_channel(dshot_channal[id-1]);
}

void DSHOT_esc_unlock(uint8_t id)
{
  switch(id)
  {
  case 1:
    dshot_send(0, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  case 2:
    dshot_send(0, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  case 3:
    dshot_send(0, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  case 4:
    dshot_send(0, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  default:
    break;
  }
}

void DSHOT_esc_rotation_set(uint8_t id, uint8_t dir)
{
  if(dir == 0)
  {
    switch(id)
    {
    case 1:
      dshot_send(7, false, dshot_channal[id-1], dshot_encoder[id-1]);
      break;
    case 2:
      dshot_send(7, false, dshot_channal[id-1], dshot_encoder[id-1]);
      break;
    case 3:
      dshot_send(7, false, dshot_channal[id-1], dshot_encoder[id-1]);
      break;
    case 4:
      dshot_send(7, false, dshot_channal[id-1], dshot_encoder[id-1]);
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
      dshot_send(8, false, dshot_channal[id-1], dshot_encoder[id-1]);
      break;
    case 2:
      dshot_send(8, false, dshot_channal[id-1], dshot_encoder[id-1]);
      break;
    case 3:
      dshot_send(8, false, dshot_channal[id-1], dshot_encoder[id-1]);
      break;
    case 4:
      dshot_send(8, false, dshot_channal[id-1], dshot_encoder[id-1]);
      break;
    default:
      break;
    }
  }
}

void DSHOT_throttle_set(uint8_t id, uint16_t throttle)
{
  if(throttle > 2047)
  {
    throttle = 2047;
  }
  else if(throttle < 48)
  {
    throttle = 48;
  }
  switch(id)
  {
  case 1:
    dshot_send(throttle, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  case 2:
    dshot_send(throttle, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  case 3:
    dshot_send(throttle, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  case 4:
    dshot_send(throttle, false, dshot_channal[id-1], dshot_encoder[id-1]);
    break;
  default:
    break;
  }
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
