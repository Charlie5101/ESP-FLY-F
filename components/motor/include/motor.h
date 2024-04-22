#ifndef MOTOR__
#define MOTOR__

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

void DSHOT_init(void);
void DSHOT_enable_channel(uint8_t id);
void DSHOT_disable_channel(uint8_t id);
void DSHOT_throttle_set(uint8_t id,uint16_t throttle);

void motor_init(void);
void servo_init(void);
uint16_t Percent_to_Resolution(float target_duty);
void motor_throttle_set(uint8_t id,float throttle);
void servo_out_set(uint8_t id,float throttle);

#endif
