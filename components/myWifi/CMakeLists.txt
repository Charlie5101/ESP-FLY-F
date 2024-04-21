idf_component_register(SRCS "myWifi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi
                    REQUIRES lwip
                    REQUIRES esp_https_ota
                    REQUIRES esp_http_client
                    REQUIRES app_update
                    REQUIRES mbedtls)
