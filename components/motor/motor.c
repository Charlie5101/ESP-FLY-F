#include <stdio.h>
#include "motor.h"

void motor_init()
{
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M1_IO,M1_CHANNEL,0,0);
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M2_IO,M2_CHANNEL,0,0);
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M3_IO,M3_CHANNEL,0,0);
  pwm_init(MOTOR_TIMER,MOTOR_RESOLUTION,MOTOR_REFRESH_RATE,M4_IO,M4_CHANNEL,0,0);

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
