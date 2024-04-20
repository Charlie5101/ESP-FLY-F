idf_component_register(SRCS "motor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bsp_pwm
                    REQUIRES DSHOT)
