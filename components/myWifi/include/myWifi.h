#ifndef MYWIFI__
#define MYWIFI__

#include "esp_wifi.h"
#include "lwip/sockets.h"

#define WIFI_REC_BUFF_LEN   128
#define MAX_RETRY_TIMES     15

typedef enum{
  SOCKET_INIT,
  SOCKET_CREATE,
  SOCKET_CONNECT,
  SOCKET_WAIT,
  SOCKET_DISCONNECT,
}Socket_State_t;

/*My Wifi Class*/
typedef struct
{
  Socket_State_t socket_service_state;
  SemaphoreHandle_t Socket_ERR;
  bool AP_CONNECT_STATE;
  struct sockaddr_in local_addr;
  char rx_buffer[WIFI_REC_BUFF_LEN];
  int addr_family;
  int ip_protocol;
  struct sockaddr_in dest_addr;
  int sock;
  uint8_t AP_retry_num;
  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

  void (*init)(void* My_Wifi);
  void (*start)(void);
  void (*stop)(void);
  struct
  {
    void (*TCP_init)(void* My_Wifi);
    void (*TCP_send)(void* My_Wifi, char *payload, uint32_t len);
    char* (*TCP_recv)(void* My_Wifi);
    void (*TCP_stop)(void* My_Wifi);
    void (*UDP_init)(void* My_Wifi);
    void (*UDP_send)(void* My_Wifi, char *payload, uint32_t len);
    char* (*UDP_recv)(void* My_Wifi);
    void (*UDP_stop)(void* My_Wifi);
  }vofa;
  void (*Socket_Service)(void* My_Wifi);
  void (*OTA)(void* My_Wifi);
}My_Wifi_Classdef;

void My_Wifi_Class_init(My_Wifi_Classdef* My_Wifi);

#endif
