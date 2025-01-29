//This code configures an ADC on GPIO36 and a PWM output on GPIO18 for the ESP32. 
// The ADC value is read and converted to a PWM duty cycle which is then applied to the PWM output.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"

// ADC Configuration
#define ADC_CHANNEL ADC1_CHANNEL_3 // GPIO39

// PWM Configuration
#define PWM_OUTPUT_PIN GPIO_NUM_23
#define PWM_FREQ 5000
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define PWM_CHANNEL LEDC_CHANNEL_0

void setup() {
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Configure PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = PWM_OUTPUT_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ledc_channel);
}

void loop() {
    // Read ADC value
    int adc_value = adc1_get_raw(ADC_CHANNEL);

    // Convert ADC value to PWM duty cycle
    int pwm_duty = (adc_value * ((1 << PWM_RESOLUTION) - 1)) / 4095;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, pwm_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);

    // Print ADC and PWM values
    printf("ADC Value: %d, PWM Duty: %d\n", adc_value, pwm_duty);
}

int main(void)
{
  setup();
  while (1) {
    loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}