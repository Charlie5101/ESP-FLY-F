#include <stdio.h>
#include "bat_adc.h"
#include "esp_log.h"

#define BAT_ADC_UNIT  ADC_UNIT_1
#define BAT_ADC_BITWIDTH  ADC_BITWIDTH_DEFAULT
#define BAT_ADC_ATTEN ADC_ATTEN_DB_12
#define BAT_ADC_CHANNEL ADC_CHANNEL_0

void Bat_ADC_init(BAT_Voltage_Classdef* BAT_Class);
void BAT_Voltage_Read(BAT_Voltage_Classdef* BAT_Class);
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

const static char *TAG = "BAT_Voltage";

void BAT_Voltage_Class_init(BAT_Voltage_Classdef* BAT_Class)
{
  BAT_Class->ADC_Handle = NULL;
  BAT_Class->ADC_Cali_Handle = NULL;
  BAT_Class->do_calibration = false;
  BAT_Class->raw_data = 0;
  BAT_Class->voltage = 0;

  BAT_Class->init = (void (*)(void*))Bat_ADC_init;
  BAT_Class->read = (void (*)(void*))BAT_Voltage_Read;

  BAT_Class->init(BAT_Class);
}

void Bat_ADC_init(BAT_Voltage_Classdef* BAT_Class)
{
  adc_oneshot_unit_init_cfg_t init_cfg = {
    .unit_id = BAT_ADC_UNIT,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &BAT_Class->ADC_Handle));

  adc_oneshot_chan_cfg_t chan_cfg = {
    .bitwidth = BAT_ADC_BITWIDTH,
    .atten = BAT_ADC_ATTEN,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(BAT_Class->ADC_Handle, BAT_ADC_CHANNEL, &chan_cfg));

  //ADC Calibration
  BAT_Class->do_calibration = adc_calibration_init(BAT_ADC_UNIT, BAT_ADC_CHANNEL, BAT_ADC_ATTEN, &BAT_Class->ADC_Cali_Handle);
}

void BAT_Voltage_Read(BAT_Voltage_Classdef* BAT_Class)
{
  ESP_ERROR_CHECK(adc_oneshot_read(BAT_Class->ADC_Handle, BAT_ADC_CHANNEL, &BAT_Class->raw_data));
//   ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", BAT_ADC_UNIT + 1, BAT_ADC_CHANNEL, BAT_Class->raw_data);
  if (BAT_Class->do_calibration) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(BAT_Class->ADC_Cali_Handle, BAT_Class->raw_data, &BAT_Class->voltage));
    // ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", BAT_ADC_UNIT + 1, BAT_ADC_CHANNEL, BAT_Class->voltage);
    BAT_Class->fvoltage = (float)(BAT_Class->voltage * 11) / 1000.0;
    ESP_LOGI(TAG, "%f", BAT_Class->fvoltage);
  }
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
