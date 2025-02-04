#include <Arduino.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include <esp_adc/adc_continuous.h>
#include <driver/gptimer.h>
#include <esp_task_wdt.h>
#include <rtc_wdt.h>

// Define the structure for ADC configuration
struct ADCConfig {
  int channel;          // ADC1 channel (e.g., ADC1_CHANNEL_3)
  int pin;              // GPIO pin number
};

// Define the GPIO pins for ADC inputs and PWM outputs
#define NUM_CHANNELS 5 // Updated to only use ADC1 channels
const ADCConfig adcConfigs[NUM_CHANNELS] = {
  {ADC1_CHANNEL_3, 39},  // GPIO 39, ADC1 channel 3
  {ADC1_CHANNEL_6, 34},  // GPIO 34, ADC1 channel 6
  {ADC1_CHANNEL_7, 35},  // GPIO 35, ADC1 channel 7
  {ADC1_CHANNEL_4, 32},  // GPIO 32, ADC1 channel 4
  {ADC1_CHANNEL_5, 33}   // GPIO 33, ADC1 channel 5
};
const int pwmPins[NUM_CHANNELS] = {23, 19, 18, 17, 16};

#define ADC_BUFFER_SIZE 256
static uint8_t adc_buffer[ADC_BUFFER_SIZE];
adc_continuous_handle_t adc_handle;

bool serialUpdate = true; // Control flag for Serial updates

gptimer_handle_t gptimer = NULL;

bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  uint32_t adc_readings[NUM_CHANNELS] = {0};
  uint32_t read_size = 0;
  
  esp_err_t ret = adc_continuous_read(adc_handle, adc_buffer, ADC_BUFFER_SIZE, &read_size, 0);
  if (ret == ESP_OK) {
    for (size_t i = 0; i < read_size; i += sizeof(adc_digi_output_data_t)) {
      adc_digi_output_data_t *data = (adc_digi_output_data_t *)&adc_buffer[i];
      int channel = data->type1.channel;
      if (channel < NUM_CHANNELS) {
        adc_readings[channel] = data->type1.data;
      }
      Serial.print(data->type1.channel);
      Serial.print(": ");
      Serial.println(data->type1.data);
    }

    for (int i = 0; i < 2; i++) {
      int adcValue = adc_readings[adcConfigs[i].channel];
      int pwmValue = map(adcValue, 0, 4095, 0, 255);

      if (serialUpdate) {
        // Debug prints
        Serial.print("ADC Value (ADC ");
        Serial.print(adcConfigs[i].pin);
        Serial.print("): ");
        Serial.println(adcValue);

        Serial.print("PWM Value (GPIO ");
        Serial.print(pwmPins[i]);
        Serial.print("): ");
        Serial.println(pwmValue);
      }

      ledc_set_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(i), pwmValue);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(i));
    }
  } else {
    Serial.print("ADC read error: ");
    Serial.println(ret);
  }

  return true; // Keep the timer repeating
}

#include "esp_system.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

void disableWatchdogTimer() {
  // Disable the Main Watchdog Timer for Timer Group 0
  TIMERG0.wdtwprotect.wdt_wkey = TIMG_WDT_WKEY_V;  // Unlock
  TIMERG0.wdtconfig0.wdt_stg0 = TIMG_WDT_STG0_HOLD;  // Disable stage 0
  TIMERG0.wdtconfig0.wdt_en = 0;                       // Disable
  TIMERG0.wdtwprotect.wdt_wkey = 0;                    // Lock

  // Disable the Main Watchdog Timer for Timer Group 1
  TIMERG1.wdtwprotect.wdt_wkey = TIMG_WDT_WKEY_V;  // Unlock
  TIMERG1.wdtconfig0.wdt_stg0 = TIMG_WDT_STG3_HOLD;  // Disable stage 0
  TIMERG1.wdtconfig0.wdt_en = 0;                       // Disable
  TIMERG1.wdtwprotect.wdt_wkey = 0;                    // Lock
}

void setup() {
  Serial.begin(115200);
  esp_task_wdt_deinit();
  rtc_wdt_protect_off();
  rtc_wdt_disable();

  disableWatchdogTimer();

  // Configure PWM using the new V3 API
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_HIGH_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_8_BIT,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = 5000,  // PWM frequency
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    ledc_channel_config_t ledc_channel = {
      .gpio_num       = pwmPins[i],
      .speed_mode     = LEDC_HIGH_SPEED_MODE,
      .channel        = static_cast<ledc_channel_t>(i),
      .intr_type      = LEDC_INTR_DISABLE,
      .timer_sel      = LEDC_TIMER_0,
      .duty           = 0, // Set initial duty cycle to 0
      .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
  }

  // Configure ADC Continuous mode
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = ADC_BUFFER_SIZE,
    .conv_frame_size = ADC_BUFFER_SIZE / sizeof(adc_digi_output_data_t),
  };
  adc_continuous_new_handle(&adc_config, &adc_handle);

  adc_continuous_config_t continuous_config = {
    .sample_freq_hz = 20000, // Adjusted sampling frequency to 20 kHz
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };

  adc_digi_pattern_config_t adc_pattern[NUM_CHANNELS];
  for (int i = 0; i < NUM_CHANNELS; i++) {
    adc_pattern[i].atten = ADC_ATTEN_DB_12;
    adc_pattern[i].bit_width = ADC_WIDTH_BIT_12;
    adc_pattern[i].channel = adcConfigs[i].channel;
    adc_pattern[i].unit = ADC_UNIT_1;
  }
  continuous_config.pattern_num = NUM_CHANNELS;
  continuous_config.adc_pattern = adc_pattern;
  adc_continuous_config(adc_handle, &continuous_config);

  // Start ADC continuous mode
  adc_continuous_start(adc_handle);

  // Configure the general-purpose timer (gptimer)
  gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000 // 1MHz resolution
  };
  gptimer_new_timer(&timer_config, &gptimer);

  gptimer_event_callbacks_t cbs = {
    .on_alarm = on_timer_alarm,
  };
  gptimer_register_event_callbacks(gptimer, &cbs, NULL);

  gptimer_alarm_config_t alarm_config = {
    .alarm_count = 50000, // Timer alarm period 50ms (20Hz frequency)
    .reload_count = 0,
    .flags = {.auto_reload_on_alarm = 1}
  };
  gptimer_set_alarm_action(gptimer, &alarm_config);

  gptimer_enable(gptimer);
  gptimer_start(gptimer);

  Serial.println("Setup completed.");
}

void loop() {
  // Toggle serialUpdate flag to stop/start serial updates based on a condition
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 's') {
      serialUpdate = !serialUpdate;
      if (serialUpdate) {
        Serial.println("Serial updates started.");
      } else {
        Serial.println("Serial updates stopped.");
      }
    }
  }

  //esp_task_wdt_reset();

  delay(100); // Small delay to minimize CPU usage
}