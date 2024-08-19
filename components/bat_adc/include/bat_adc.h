#ifndef BAT_ADC__
#define BAT_ADC__

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

typedef struct{
  adc_oneshot_unit_handle_t ADC_Handle;
  adc_cali_handle_t ADC_Cali_Handle;
  bool do_calibration;

  int raw_data;
  int voltage;
  float fvoltage;

  void (*init)(void* BAT_Class);
  void (*read)(void *BAT_Class);
}BAT_Voltage_Classdef;

void BAT_Voltage_Class_init(BAT_Voltage_Classdef* BAT_Class);

#endif
