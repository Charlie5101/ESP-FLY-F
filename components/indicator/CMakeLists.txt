idf_component_register(SRCS "indicator.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bsp_rmt
                    REQUIRES bsp_spi)
