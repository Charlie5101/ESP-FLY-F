idf_component_register(SRCS "sensor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bsp_spi
                    REQUIRES algorithm)
