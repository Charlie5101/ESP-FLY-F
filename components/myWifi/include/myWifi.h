#ifndef MYWIFI__
#define MYWIFI__

#include "esp_wifi.h"

void myWifi_init(void);
void myWifi_start(void);
void my_wifi_vofa_init();
void myWifi_vofa_send(char *payload,uint32_t len);

#endif
