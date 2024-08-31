#include <stdio.h>
#include "myWifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef enum{
  SOCKET_INIT,
  SOCKET_CREATE,
  SOCKET_CONNECT,
  SOCKET_WAIT,
  SOCKET_DISCONNECT,
}Socket_State_t;

Socket_State_t socket_service_state;
SemaphoreHandle_t Socket_ERR;

bool AP_CONNECT_STATE = false;

static const char *TAG = "My Wifi";

#define Port        1347
#define UDP_Port    3333
#define LOCAL_PORT  1346
// #define UDP_HOST_IP "192.168.137.1"
#define UDP_HOST_IP "255.255.255.255"
struct sockaddr_in local_addr;

char rx_buffer[WIFI_REC_BUFF_LEN];
char host_ip[] = "192.168.137.1";
int addr_family = 0;
int ip_protocol = 0;
struct sockaddr_in dest_addr;
int sock;

uint8_t AP_retry_num = 0;

esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;

#define HASH_LEN 32
// extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");

void event_handler(void *arg,esp_event_base_t event_base,int32_t event_id,void *event_data)
{
  if(event_base == WIFI_EVENT)
  {
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      // if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
      //       esp_wifi_connect();
      //       s_retry_num++;
      //       ESP_LOGI(TAG, "retry to connect to the AP");
      //   } else {
      //       xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      //   }
      if(AP_retry_num < MAX_RETRY_TIMES)
      {
        esp_wifi_connect();
        AP_retry_num++;
        ESP_LOGW(TAG,"Retry connect to the AP......");
      }
      else
      {
        ESP_LOGW(TAG,"connect to the AP fail");
      }
      break;
    default:
      break;
    }
  }
  else if(event_base == IP_EVENT)
  {
    switch (event_id)
    {
    case IP_EVENT_STA_GOT_IP:
      ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
      ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
      // s_retry_num = 0;
      // xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      AP_CONNECT_STATE = true;
      break;
    default:
      break;
    }
  }
}

void myWifi_init(void)
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &event_handler,
                                                      NULL,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &event_handler,
                                                      NULL,
                                                      &instance_got_ip));
  wifi_init_config_t mywifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&mywifi_init_cfg));
  wifi_config_t mywifi_cfg = {
    .sta = {
      .ssid = "Elfnet",
      .password = "aabb8899",
      .threshold.authmode = WIFI_AUTH_WPA2_PSK,
      .pmf_cfg = {
        .capable = true,
        .required = false
      },
    },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA,&mywifi_cfg));
}

void myWifi_start(void)
{
  ESP_ERROR_CHECK(esp_wifi_start());
}

void myWifi_stop(void)
{
  ESP_ERROR_CHECK(esp_wifi_stop());
}

void Socket_Service(void)
{
  switch (socket_service_state)
  {
  case SOCKET_INIT:
    ESP_LOGI("Socket_Service:", "SOCKET_INIT");
    //wait WIFI AP connect
    while(AP_CONNECT_STATE != true)
    {
      vTaskDelay(500);
    }
    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(Port);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    socket_service_state = SOCKET_CREATE;
    break;

  case SOCKET_CREATE:
    ESP_LOGI("Socket_Service:", "SOCKET_CREATE");
    sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
    while(sock < 0)
    {
      ESP_LOGW(TAG, "Unable to create socket: errno %d", errno);
      vTaskDelay(1000);
      ESP_LOGW(TAG, "Retry to create socket......");
      sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
    }
    ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, Port);

    socket_service_state = SOCKET_CONNECT;
    break;

  case SOCKET_CONNECT:
    ESP_LOGI("Socket_Service:", "SOCKET_CONNECT");
    int err;
    err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    while(err != 0)
    {
      ESP_LOGW(TAG, "Socket unable to connect: errno %d", errno);
      vTaskDelay(1000);
      ESP_LOGW(TAG, "Socket Retry to connect......");
      close(sock);
      vTaskDelay(100);
      sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
      err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }
    ESP_LOGI(TAG, "Successfully connected");

    socket_service_state = SOCKET_WAIT;
    break;

  case SOCKET_WAIT:
    if(xSemaphoreTake(Socket_ERR,0) == pdTRUE)
    {
      socket_service_state = SOCKET_DISCONNECT;
    }
    break;

  case SOCKET_DISCONNECT:
    ESP_LOGI("Socket_Service:", "SOCKET_DISCONNECT");
    socket_service_state = SOCKET_INIT;
    break;
  default:
    break;
  }
}

void my_wifi_TCP_vofa_init(void)
{
  Socket_ERR = xSemaphoreCreateBinary();
  if(Socket_ERR == NULL)
  {
    ESP_LOGE(TAG, "Socket_ERR Semap Create Fail");
  }
  //wait WIFI AP connect
  while(AP_CONNECT_STATE != true)
  {
    vTaskDelay(500);
  }

  inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(Port);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;

  int err;

  sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
  while(sock < 0)
  {
    ESP_LOGW(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelay(1000);
    ESP_LOGW(TAG, "Retry to create socket......");
    sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
  }
  ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, Port);

  err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  while(err != 0)
  {
    ESP_LOGW(TAG, "Socket unable to connect: errno %d", errno);
    vTaskDelay(1000);
    ESP_LOGW(TAG, "Socket Retry to connect......");
    close(sock);
    vTaskDelay(100);
    sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
    err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  }
  ESP_LOGI(TAG, "Successfully connected");

  socket_service_state = SOCKET_WAIT;
}

void myWifi_Socket_Re_Connect(void)
{
  int err;
  err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  while(err != 0)
  {
    ESP_LOGW(TAG, "Socket unable to connect: errno %d", errno);
    vTaskDelay(1000);
    ESP_LOGW(TAG, "Socket Retry to connect......");
  }
  ESP_LOGI(TAG, "Successfully connected");
}

void myWifi_TCP_vofa_send(char *payload,uint32_t len)
{
  if(socket_service_state == SOCKET_WAIT)
  {
    int err = send(sock, payload, len, 0);
    if (err < 0) {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      if(socket_service_state == SOCKET_WAIT)
      {
        xSemaphoreGive(Socket_ERR);
        ESP_LOGW(TAG, "Socket_ERR Give");
      }
    }
  }
  // int err = send(sock, payload, strlen(payload), 0);
  // int err = send(sock, payload, len, 0);
  // if (err < 0) {
  //   ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
  //   if(socket_service_state == SOCKET_WAIT)
  //   {
  //     xSemaphoreGive(Socket_ERR);
  //     ESP_LOGW(TAG, "Socket_ERR Give");
  //   }
  // }

  // if (sock != -1) {
  //   ESP_LOGE(TAG, "Shutting down socket and restarting...");
  //   shutdown(sock, 0);
  //   close(sock);
  // }
}

char* myWifi_TCP_vofa_recv(void)
{
  while(socket_service_state != SOCKET_WAIT)
  {
    vTaskDelay(1000);
  }
  int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
  // Error occurred during receiving
  if (len < 0) {
    ESP_LOGE(TAG, "recv failed: errno %d", errno);
    if(socket_service_state == SOCKET_WAIT)
      xSemaphoreGive(Socket_ERR);
  }
  // Data received
  else {
    rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
    ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
    ESP_LOGI(TAG, "%s", rx_buffer);
    return rx_buffer;
  }

  // if (sock != -1) {
  //   ESP_LOGE(TAG, "Shutting down socket and restarting...");
  //   shutdown(sock, 0);
  //   close(sock);
  // }
  return rx_buffer;
}

void myWifi_TCP_vofa_stop(void)
{
  shutdown(sock,0);
  int err = close(sock);
  if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during clossing: errno %d", errno);
    while(1)
    {
      vTaskDelay(1000);
    }
  }
  ESP_LOGW(TAG, "Vofa Socket close");
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void)
{
    uint8_t sha_256[HASH_LEN] = { 0 };
    esp_partition_t partition;

    // get sha256 digest for bootloader
    partition.address   = ESP_BOOTLOADER_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_OFFSET;
    partition.type      = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
}

void OTA_update(void)
{
  //Full power wifi
  esp_wifi_set_ps(WIFI_PS_NONE);
  //init
  get_sha256_of_partitions();

  esp_http_client_config_t http_config = {
    .url = "http://192.168.137.1:8040/build/ESP-FLY.bin",
    // .cert_pem = (char *)server_cert_pem_start,
    .crt_bundle_attach = esp_crt_bundle_attach,
    .event_handler = _http_event_handler,
    .keep_alive_enable = true,
  };
  esp_https_ota_config_t ota_config = {
    .http_config = &http_config,
  };
  ESP_LOGI(TAG, "Attempting to download update from %s", http_config.url);
  esp_err_t ret = esp_https_ota(&ota_config);
  if (ret == ESP_OK) {
      ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
      esp_restart();
    } else {
      ESP_LOGE(TAG, "Firmware upgrade failed");
  }
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void my_wifi_UDP_vofa_init(void)
{
  Socket_ERR = xSemaphoreCreateBinary();
  if(Socket_ERR == NULL)
  {
    ESP_LOGE(TAG, "Socket_ERR Semap Create Fail");
  }
  //wait WIFI AP connect
  while(AP_CONNECT_STATE != true)
  {
    vTaskDelay(500);
  }

  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;

  sock =  socket(addr_family, SOCK_DGRAM, ip_protocol);
  while(sock < 0)
  {
    ESP_LOGW(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelay(1000);
    ESP_LOGW(TAG, "Retry to create socket......");
    sock =  socket(addr_family, SOCK_DGRAM, ip_protocol);
  }

  dest_addr.sin_family = AF_INET;
  dest_addr.sin_addr.s_addr = inet_addr(UDP_HOST_IP);
  dest_addr.sin_port = htons(UDP_Port);

  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(LOCAL_PORT);
  uint8_t res = 0;
  res = bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr));
  if(res != 0)
  {
    ESP_LOGW(TAG, "Bind Err");
  }

  ESP_LOGI(TAG, "Socket created, Config to %s:%d", UDP_HOST_IP, UDP_Port);

  socket_service_state = SOCKET_WAIT;
}

void myWifi_UDP_vofa_send(char *payload,uint32_t len)
{
  if(socket_service_state == SOCKET_WAIT)
  {
    // int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    int err = sendto(sock, payload, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if(err < 0)
    {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      if(socket_service_state == SOCKET_WAIT)
      {
        xSemaphoreGive(Socket_ERR);
        ESP_LOGW(TAG, "Socket_ERR Give");
      }
    }
  }
}

char* myWifi_UDP_vofa_recv(void)
{
  while(socket_service_state != SOCKET_WAIT)
  {
    vTaskDelay(1000);
  }
  struct sockaddr_storage source_addr;
  socklen_t socklen = sizeof(source_addr);
  int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);

  // Error occurred during receiving
  if (len < 0) {
    ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
    if(socket_service_state == SOCKET_WAIT)
      xSemaphoreGive(Socket_ERR);
  }
  // Data received
  else {
    rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
    ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
    ESP_LOGI(TAG, "%s", rx_buffer);
    return rx_buffer;
  }

  // if (sock != -1) {
  //   ESP_LOGE(TAG, "Shutting down socket and restarting...");
  //   shutdown(sock, 0);
  //   close(sock);
  // }
  return rx_buffer;
}

void myWifi_UDP_vofa_stop(void)
{
  shutdown(sock,0);
  int err = close(sock);
  if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during clossing: errno %d", errno);
    while(1)
    {
      vTaskDelay(1000);
    }
  }
  ESP_LOGW(TAG, "Vofa Socket close");
}
