idf_component_register(SRCS "myWifi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi
                    REQUIRES lwip)
