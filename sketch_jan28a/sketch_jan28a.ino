#include <Arduino.h>
#include <driver/adc.h>
#include <driver/ledc.h>

// Define the structure for ADC configuration
struct ADCConfig {
  adc_unit_t unit;      // ADC unit (ADC_UNIT_1 or ADC_UNIT_2)
  int channel;          // ADC channel (e.g., ADC1_CHANNEL_3)
  int pin;              // GPIO pin number
};

// Define the GPIO pins for ADC inputs and PWM outputs
#define NUM_CHANNELS 6
const ADCConfig adcConfigs[NUM_CHANNELS] = {
  {ADC_UNIT_1, ADC1_CHANNEL_3, 39},  // GPIO 39, ADC1 channel 3
  {ADC_UNIT_1, ADC1_CHANNEL_6, 34},  // GPIO 34, ADC1 channel 6
  {ADC_UNIT_1, ADC1_CHANNEL_7, 35},  // GPIO 35, ADC1 channel 7
  {ADC_UNIT_1, ADC1_CHANNEL_4, 32},  // GPIO 32, ADC1 channel 4
  {ADC_UNIT_1, ADC1_CHANNEL_5, 33},  // GPIO 33, ADC1 channel 5
  {ADC_UNIT_2, ADC2_CHANNEL_8, 25}   // GPIO 25, ADC2 channel 8
};
const int pwmPins[NUM_CHANNELS] = {23, 19, 18, 17, 16, 4};

void setup() {
  Serial.begin(115200);

  // Configure ADC channels
  adc1_config_width(ADC_WIDTH_BIT_12); // Set ADC1 resolution to 12 bits (0-4095)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (adcConfigs[i].unit == ADC_UNIT_1) {
      adc1_config_channel_atten((adc1_channel_t)adcConfigs[i].channel, ADC_ATTEN_DB_11); // Set attenuation to 11dB for full range (0-3.6V)
    } else {
      adc2_config_channel_atten((adc2_channel_t)adcConfigs[i].channel, ADC_ATTEN_DB_11); // Set attenuation to 11dB for full range (0-3.6V)
    }
  }

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

  Serial.println("Setup completed.");
}

void loop() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    int adcValue;

    // Read ADC value from the corresponding input pin
    if (adcConfigs[i].unit == ADC_UNIT_1) {
      // Use ADC1 channels
      adcValue = adc1_get_raw((adc1_channel_t)adcConfigs[i].channel);
    } else {
      // Use ADC2 for the last channel
      adc2_get_raw((adc2_channel_t)adcConfigs[i].channel, ADC_WIDTH_BIT_12, &adcValue);
    }

    Serial.print("ADC Value (GPIO ");
    Serial.print(adcConfigs[i].pin);
    Serial.print("): ");
    Serial.println(adcValue);

    // Map the ADC value to PWM range (0-255)
    int pwmValue = map(adcValue, 0, 4095, 0, 255);
    Serial.print("PWM Value (GPIO ");
    Serial.print(pwmPins[i]);
    Serial.print("): ");
    Serial.println(pwmValue);

    // Set the PWM value on the corresponding output pin using the new V3 API
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(i), pwmValue);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(i));
  }

  delay(100);
}