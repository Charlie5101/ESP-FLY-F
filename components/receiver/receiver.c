#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "receiver.h"

#define RECEIVER_UART uart_2
#define REC_TX        UART2_TX_PIN
#define REC_RX        UART2_RX_PIN

#ifdef CRSF_RECEIVER 
#define REC_BAUDRATE 420000
#endif

void Receiver_init(Receiver_Classdef* Receiver);
void Receiver_rec_data(Receiver_Classdef* Receiver, uint8_t* pdata, uint16_t len);
void CRSF_crc8_init(Receiver_Classdef* Receiver);
uint8_t CRSF_crc8_check(Receiver_Classdef* Receiver, uint8_t* pdata);
void CRSF_decode(Receiver_Classdef* Receiver);

void Receiver_Class_init(Receiver_Classdef* Receiver)
{
  Receiver->main_data.ch0 = Receiver->main_data.ch1 = Receiver->main_data.ch3 = Receiver->main_data.ch4 =
  Receiver->main_data.ch5 = Receiver->main_data.ch6 = Receiver->main_data.ch7 = Receiver->main_data.ch8 =
  Receiver->main_data.ch9 = Receiver->main_data.ch10 = Receiver->main_data.ch11 = Receiver->main_data.ch12 =
  Receiver->main_data.ch13 = Receiver->main_data.ch14 = Receiver->main_data.ch15 = 0.0f;
  bzero(Receiver->dtmp, REC_BUFF_LEN);
  Receiver->dlen = 0;
  Receiver->LinkState = false;
  Receiver->LinkNum = 0;
  bzero(Receiver->crsf.crc8_table, 256);
  bzero(&Receiver->crsf.LinkInfo, LinkStatisticsFrameLength);

  Receiver->init = (void (*)(void*))Receiver_init;
  Receiver->rec_data = (void (*)(void*, uint8_t* pdata, uint16_t len))Receiver_rec_data;
  Receiver->crsf.crc8_init = (void (*)(void*))CRSF_crc8_init;
  Receiver->crsf.crc_check = (uint8_t (*)(void*, uint8_t* pdata))CRSF_crc8_check;
  Receiver->crsf.decode = (void (*)(void*))CRSF_decode;

  Receiver->init(Receiver);
}

void Receiver_init(Receiver_Classdef* Receiver)
{
  uart_init(RECEIVER_UART, REC_BAUDRATE, REC_TX, REC_RX, 1024 * 2, 1024 * 2, 20, &Receiver->queue);
  // uart_intr_cfg(RECEIVER_UART, 0x80, 2, 20, 0);     //Enable UART_BRK_DET_INT
  // uart_disable_rx_intr(RECEIVER_UART);
  // uart_intr_cfg(RECEIVER_UART, 128, 10, 20, 0);
  // uart_rx_intr_enable(RECEIVER_UART);
  // uart_enable_intr_mask(RECEIVER_UART, 128);
  // uart_enable_rx_intr(RECEIVER_UART);
  uart_intr_event_serve_create(&Uart2_data_rec, (TaskFunction_t)uart_event_task, &Receiver->queue, &Uart2_event_Handle);
}

void Receiver_rec_data(Receiver_Classdef* Receiver, uint8_t* pdata, uint16_t len)
{
  Receiver->dlen = len;
  memcpy(&Receiver->dtmp, pdata, len);
}

void CRSF_crc8_init(Receiver_Classdef* Receiver)
{
  for(uint16_t i=0; i<256; i++)
  {
    uint8_t crc = i;
    for(uint8_t j=0; j<8; j++)
    {
      crc = ( crc << 1 ) ^ ( ( crc & 0x80 ) ? CRSF_CRC_POLY : 0);
    }
    Receiver->crsf.crc8_table[i] = crc & 0xff;
  }
}

uint8_t CRSF_crc8_check(Receiver_Classdef* Receiver, uint8_t* pdata)
{
  Receiver->crsf.len = Receiver->dtmp[1];
  uint8_t len = Receiver->crsf.len - 1;
  Receiver->crsf.inCrc = Receiver->dtmp[2 + Receiver->crsf.len - 1];
  uint8_t crc = 0;
  while(len--)
  {
    crc = Receiver->crsf.crc8_table[crc ^ *pdata++];
  }
  if(crc == Receiver->crsf.inCrc)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void CRSF_decode(Receiver_Classdef* Receiver)
{
  const crsf_header_t *hdr = (crsf_header_t*)&Receiver->dtmp[0];
  switch (hdr->device_addr)
  {
    case CRSF_ADDRESS_FLIGHT_CONTROLLER:
      switch (hdr->type)
      {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
          rcPacket_t *rcPack = (rcPacket_t*)&Receiver->dtmp[0];
          Receiver->main_data.ch0 = ( (float)rcPack->channels.ch0 - 1024 ) / 1024;
          Receiver->main_data.ch1 = ( (float)rcPack->channels.ch1 - 1024 ) / 1024;
          Receiver->main_data.ch2 = ( (float)rcPack->channels.ch2 ) / 2048;  //throttle
          Receiver->main_data.ch3 = ( (float)rcPack->channels.ch3 - 1024 ) / 1024;
          Receiver->main_data.ch4 = ( (float)rcPack->channels.ch4 - 1024 ) / 1024;
          Receiver->main_data.ch5 = ( (float)rcPack->channels.ch5 - 1024 ) / 1024;
          Receiver->main_data.ch6 = ( (float)rcPack->channels.ch6 - 1024 ) / 1024;
          Receiver->main_data.ch7 = ( (float)rcPack->channels.ch7 - 1024 ) / 1024;
          Receiver->main_data.ch8 = ( (float)rcPack->channels.ch8 - 1024 ) / 1024;
          Receiver->main_data.ch9 = ( (float)rcPack->channels.ch9 - 1024 ) / 1024;
          Receiver->main_data.ch10 = ( (float)rcPack->channels.ch10 - 1024 ) / 1024;
          Receiver->main_data.ch11 = ( (float)rcPack->channels.ch11 - 1024 ) / 1024;
          Receiver->main_data.ch12 = ( (float)rcPack->channels.ch12 - 1024 ) / 1024;
          Receiver->main_data.ch13 = ( (float)rcPack->channels.ch13 - 1024 ) / 1024;
          Receiver->main_data.ch14 = ( (float)rcPack->channels.ch14 - 1024 ) / 1024;
          Receiver->main_data.ch15 = ( (float)rcPack->channels.ch15 - 1024 ) / 1024;
          break;
        case CRSF_FRAMETYPE_GPS:
          break;
        case CRSF_FRAMETYPE_VARIO:
          break;
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
          break;
        case CRSF_FRAMETYPE_BARO_ALTITUDE:
          break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
          crsfLinkStatistics_t *LinkPack = (crsfLinkStatistics_t*)&Receiver->dtmp[3];
          memcpy(&Receiver->crsf.LinkInfo, LinkPack, LinkStatisticsFrameLength);
          // ESP_LOGI("RECEIVER","-------------------------------------------------------------");
          // ESP_LOGI("RECEIVER","UPLink RSSI 1:  %d",LinkPack->uplink_RSSI_1);
          // ESP_LOGI("RECEIVER","UPLink RSSI 2:  %d",LinkPack->uplink_RSSI_2);
          // ESP_LOGI("RECEIVER","UPLink Link Quality:  %d",LinkPack->uplink_Link_quality);
          // ESP_LOGI("RECEIVER","UPLink SNR:  %d",LinkPack->uplink_SNR);
          // ESP_LOGI("RECEIVER","Active Antenna:  %d",LinkPack->active_antenna);
          // ESP_LOGI("RECEIVER","RF Mode:  %d",LinkPack->rf_Mode);
          // ESP_LOGI("RECEIVER","UPLink TX Power:  %d",LinkPack->uplink_TX_Power);
          // ESP_LOGI("RECEIVER","DownLink RSSI:  %d",LinkPack->downlink_RSSI);
          // ESP_LOGI("RECEIVER","DownLink Link Quality:  %d",LinkPack->downlink_Link_quality);
          // ESP_LOGI("RECEIVER","DownLink SNR:  %d",LinkPack->downlink_SNR);
          // ESP_LOGI("RECEIVER","-------------------------------------------------------------");
          break;
        case CRSF_FRAMETYPE_OPENTX_SYNC:
          break;
        case CRSF_FRAMETYPE_RADIO_ID:
          break;
        case CRSF_FRAMETYPE_ATTITUDE:
          break;
        case CRSF_FRAMETYPE_FLIGHT_MODE:
          break;
        default:
          break;
      }
      break;
    case CRSF_ADDRESS_BROADCAST:
      break;
    case CRSF_ADDRESS_USB:
      break;
    case CRSF_ADDRESS_TBS_CORE_PNP_PRO:
      break;
    case CRSF_ADDRESS_RESERVED1:
      break;
    case CRSF_ADDRESS_CURRENT_SENSOR:
      break;
    case CRSF_ADDRESS_GPS:
      break;
    case CRSF_ADDRESS_TBS_BLACKBOX:
      break;
    case CRSF_ADDRESS_RESERVED2:
      break;
    case CRSF_ADDRESS_RACE_TAG:
      break;
    case CRSF_ADDRESS_RADIO_TRANSMITTER:
      break;
    case CRSF_ADDRESS_CRSF_RECEIVER:
      break;
    case CRSF_ADDRESS_CRSF_TRANSMITTER:
      break;
    case CRSF_ADDRESS_ELRS_LUA:
      break;
    default:
      break;
  }
}
