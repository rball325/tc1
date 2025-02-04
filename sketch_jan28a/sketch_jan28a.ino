#include <Arduino.h>
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

#include "utility.h"

// Define a structure for ADC configuration
struct AdcConfig {
  adc_unit_t unit;
  adc_channel_t channel;
  int gpio_output;
  ledc_channel_t pwm_channel;
};

// Define ADC channels and corresponding GPIO outputs
AdcConfig adcPins[] = {
  { ADC_UNIT_1, ADC_CHANNEL_3, 23, LEDC_CHANNEL_0 }, // GPIO 39, Output GPIO 23
  { ADC_UNIT_1, ADC_CHANNEL_6, 19, LEDC_CHANNEL_1 }, // GPIO 34, Output GPIO 19
  { ADC_UNIT_1, ADC_CHANNEL_7, 18, LEDC_CHANNEL_2 }, // GPIO 35, Output GPIO 18
  { ADC_UNIT_1, ADC_CHANNEL_4, 17, LEDC_CHANNEL_3 }, // GPIO 32, Output GPIO 17
  { ADC_UNIT_1, ADC_CHANNEL_5, 16, LEDC_CHANNEL_4 }, // GPIO 33, Output GPIO 16
  { ADC_UNIT_2, ADC_CHANNEL_8,  4, LEDC_CHANNEL_5 }  // GPIO 25, Output GPIO 4
};

const int numAdcPins = sizeof(adcPins) / sizeof(adcPins[0]);

// ADC configuration
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;

// PWM configuration
const int pwmFreq = 15000;  // PWM frequency
const ledc_timer_bit_t pwmResolution = LEDC_TIMER_8_BIT;  // PWM resolution (8-bit)
const int pwmMaxValue = (1 << pwmResolution) - 1;  // Max PWM value (255 for 8-bit)

void setup() {
  Serial.begin(115200);
  showResetReason();

  // Initialize ADC configuration
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
  };
  adc_oneshot_new_unit(&init_config1, &adc1_handle);

  adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2,
    .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
  };
  adc_oneshot_new_unit(&init_config2, &adc2_handle);

  // Configure each ADC channel
  for (int i = 0; i < numAdcPins; i++) {
    adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12, // Choose appropriate attenuation level
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adcPins[i].unit == ADC_UNIT_1) {
      adc_oneshot_config_channel(adc1_handle, adcPins[i].channel, &config);
    } else {
      adc_oneshot_config_channel(adc2_handle, adcPins[i].channel, &config);
    }
  }

  // Configure PWM timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_HIGH_SPEED_MODE,
    .duty_resolution  = pwmResolution,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = pwmFreq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Initialize PWM channels
  for (int i = 0; i < numAdcPins; i++) {
    ledc_channel_config_t ledc_channel = {
      .gpio_num       = adcPins[i].gpio_output,
      .speed_mode     = LEDC_HIGH_SPEED_MODE,
      .channel        = adcPins[i].pwm_channel,
      .intr_type      = LEDC_INTR_DISABLE,
      .timer_sel      = LEDC_TIMER_0,
      .duty           = 0,
      .hpoint         = 0,
    };
    ledc_channel_config(&ledc_channel);
  }

  Serial.println("ADC and PWM initialized.");
}

void loop() {
  // Perform one-shot ADC reads for each channel
  int adc_values[numAdcPins];
  for (int i = 0; i < numAdcPins; i++) {
    if (adcPins[i].unit == ADC_UNIT_1) {
      adc_oneshot_read(adc1_handle, adcPins[i].channel, &adc_values[i]);
    } else {
      adc_oneshot_read(adc2_handle, adcPins[i].channel, &adc_values[i]);
    }

    // Map the ADC value to the PWM range
    int pwm_value = map(adc_values[i], 0, 4095, 0, pwmMaxValue);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, adcPins[i].pwm_channel, pwm_value);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, adcPins[i].pwm_channel);

    // Print the ADC value and corresponding PWM value
#if 0
    Serial.print("ADC ");
    Serial.print(i + 1);
    Serial.print(" (OUT ");
    Serial.print(adcPins[i].gpio_output);
    Serial.print("): ");
    Serial.print(adc_values[i]);
    Serial.print(", PWM Value: ");
    Serial.println(pwm_value);
  }
  delay(1000); // ms
#else
  }
  delay(10); // ms
#endif
}
