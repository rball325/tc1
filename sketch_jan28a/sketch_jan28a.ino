#include <Arduino.h>
#include <driver/adc.h>
#include <driver/ledc.h>

#define ADC_PIN 39
#define PWM_PIN 23

void setup() {
  Serial.begin(115200);

  // Configure ADC1 Channel 3 (GPIO 39)
  adc1_config_width(ADC_WIDTH_BIT_12); // Set ADC resolution to 12 bits (0-4095)
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); // Set attenuation to 11dB for full range (0-3.6V)

  // Configure PWM on GPIO 23 using the new V3 API
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_HIGH_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_8_BIT,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = 5000,  // PWM frequency
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .gpio_num       = PWM_PIN,
    .speed_mode     = LEDC_HIGH_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0, // Set initial duty cycle to 0
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel);

  Serial.println("Setup completed.");
}

void loop() {
  // Read ADC value from GPIO 39
  int adcValue = adc1_get_raw(ADC1_CHANNEL_3);
  Serial.print("ADC Value: ");
  Serial.println(adcValue);

  // Map the ADC value to PWM range (0-255)
  int pwmValue = map(adcValue, 0, 4095, 0, 255);
  Serial.print("PWM Value: ");
  Serial.println(pwmValue);

  // Set the PWM value on GPIO 23 using the new V3 API
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pwmValue);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

  delay(100);
}