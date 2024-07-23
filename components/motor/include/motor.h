#ifndef MOTOR__
#define MOTOR__

#include "DSHOT.h"

#define MOTOR_REFRESH_RATE        300
#define MOTOR_RESOLUTION          LEDC_TIMER_14_BIT
#define MOTOR_TIMER               LEDC_TIMER_0
#define M1_CHANNEL                LEDC_CHANNEL_0
#define M2_CHANNEL                LEDC_CHANNEL_1
#define M3_CHANNEL                LEDC_CHANNEL_2
#define M4_CHANNEL                LEDC_CHANNEL_3
#define M1_IO                     42
#define M2_IO                     41
#define M3_IO                     40
#define M4_IO                     39

#define SERVO_REFRESH_RATE        50
#define SERVO_RESOLUTION          LEDC_TIMER_14_BIT
#define SERVO_TIMER               LEDC_TIMER_1
#define S1_CHANNEL                LEDC_CHANNEL_4
#define S2_CHANNEL                LEDC_CHANNEL_5
#define S3_CHANNEL                LEDC_CHANNEL_6
#define S4_CHANNEL                LEDC_CHANNEL_7
#define S1_IO                     38
#define S2_IO                     4
#define S3_IO                     5
#define S4_IO                     6

typedef struct
{
  void (*init)(void);
  uint16_t (*Per_to_Res)(float target_duty);
  void (*Set_Throttle)(uint8_t id, float throttle);
}Motor_Classdef;

typedef struct
{
  int8_t motor_io[4];
  DSHOT_Channel_handle_t dshot_channal[4];
  DSHOT_encoder_handle_t dshot_encoder[4];

  void (*init)(void* DSHOT);
  void (*channel_enable)(void* DSHOT, uint8_t id);
  void (*channel_disable)(void* DSHOT, uint8_t id);
  void (*ESC_unLock)(void* DSHOT, uint8_t id);
  void (*ESC_Rotation_Set)(void* DSHOT, uint8_t id, uint8_t dir);
  void (*Set_Throttle)(void* DSHOT, uint8_t id, uint16_t throttle);
  void (*Enter_Set)(void* DSHOT, uint8_t id);
  void (*Exit_and_Save_Set)(void* DSHOT, uint8_t id);
  void (*ESC_Rotation_Reverse)(void* DSHOT, uint8_t id);
  void (*Set_All_Throttle)(void* DSHOT, uint16_t throttle_A, uint16_t throttle_B, uint16_t throttle_C, uint16_t throttle_D);
}DSHOT_Classdef;

typedef struct
{
  void (*init)(void);
  uint16_t (*Per_to_Res)(float target_duty);
  void (*Set_Out)(uint8_t id, float throttle);
}Servo_Classdef;

void Motor_Class_init(Motor_Classdef* Motor);
void DSHOT_Class_init(DSHOT_Classdef* DSHOT);
void Servo_Class_init(Servo_Classdef* Servo);
/*
void DSHOT_init(DSHOT_Classdef* DSHOT);
void DSHOT_enable_channel(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_disable_channel(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_esc_unlock(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_esc_rotation_set(DSHOT_Classdef* DSHOT, uint8_t id, uint8_t dir);
void DSHOT_throttle_set(DSHOT_Classdef* DSHOT, uint8_t id, uint16_t throttle);
void DSHOT_enter_set(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_exit_and_save_set(DSHOT_Classdef* DSHOT, uint8_t id);
void DSHOT_esc_rotation_reverse(DSHOT_Classdef* DSHOT, uint8_t id);

void motor_init(void);
void servo_init(void);
uint16_t Percent_to_Resolution(float target_duty);
void motor_throttle_set(uint8_t id,float throttle);
void servo_out_set(uint8_t id,float throttle);
*/

#endif
