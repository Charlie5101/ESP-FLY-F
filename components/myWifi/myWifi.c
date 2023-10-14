#include <stdio.h>
#include "myWifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

static const char *TAG = "wifi station";

#define Port        1347
char rx_buffer[128];
char host_ip[] = "192.168.137.1";
int addr_family = 0;
int ip_protocol = 0;
struct sockaddr_in dest_addr;
int sock;

esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;

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
        ESP_LOGI(TAG,"connect to the AP fail");
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

void my_wifi_vofa_init()
{
  static const char *payload = "Message from ESP32 \n";
  inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(Port);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;

  int err;

  sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    while(1)
    {
      vTaskDelay(100);
    }
  }
  ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, Port);

  err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
    while(1)
    {
      vTaskDelay(100);
    }
  }
  ESP_LOGI(TAG, "Successfully connected");

  err = send(sock, payload, strlen(payload), 0);
  if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    while(1)
    {
      vTaskDelay(100);
    }
  }

  // int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
  // // Error occurred during receiving
  // if (len < 0) {
  //   ESP_LOGE(TAG, "recv failed: errno %d", errno);
  //   while(1)
  //   {
  //     vTaskDelay(100);
  //   }
  // }
  // // Data received
  // else {
  //   rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
  //   ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
  //   ESP_LOGI(TAG, "%s", rx_buffer);
  // }

  // if (sock != -1) {
  //   ESP_LOGE(TAG, "Shutting down socket and restarting...");
  //   shutdown(sock, 0);
  //   close(sock);
  // }
}

void myWifi_vofa_send(char *payload,uint32_t len)
{
  // int err = send(sock, payload, strlen(payload), 0);
  int err = send(sock, payload, len, 0);
  if (err < 0) {
    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    while(1)
    {
      vTaskDelay(1000);
    }
  }

  // if (sock != -1) {
  //   ESP_LOGE(TAG, "Shutting down socket and restarting...");
  //   shutdown(sock, 0);
  //   close(sock);
  // }
}
