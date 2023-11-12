#ifndef MYWIFI__
#define MYWIFI__

#include "esp_wifi.h"

#define WIFI_REC_BUFF_LEN   128

void myWifi_init(void);
void myWifi_start(void);
void my_wifi_vofa_init(void);
void myWifi_vofa_send(char *payload,uint32_t len);
char* myWifi_vofa_recv(void);
void myWifi_vofa_stop(void);

void OTA_update(void);

#endif
