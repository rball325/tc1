#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

// Constants
const int RUN_TIME_SHORT = 10;  // Define the short run time in seconds
const int RUN_TIME_LONG = 30;   // Define the long run time in seconds
const int ACCELERATION_TIME = 5000; // Time to accelerate/decelerate in milliseconds
const int PWM_FREQ = 15000;     // PWM frequency in Hz

// GPIO Pins
const int sw1aPin = 27;
const int sw1bPin = 26;
const int sw3Pin = 13;
const int mosfet7Pin = 2;
const int pwmPins[] = {23, 19, 18, 17, 16, 4};
const int potPins[] = {39, 34, 35, 32, 33, 25};

// PWM Configuration
const int pwmResolution = 8;   // Resolution of PWM (8-bit)
const int pwmMaxValue = 255;   // Maximum duty cycle value (for 8-bit resolution)
const int pwmChannels[] = {0, 1, 2, 3, 4, 5, 6}; // Including channel for MOSFET7

// ADC Channels
const int adc1Channels[] = {ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_4, ADC_CHANNEL_5};
const int adc2Channel = ADC_CHANNEL_8;

// ADC Configuration
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;

// Variables
bool trainRunning = false;
int idleTrack = 0;
unsigned long startTime;
unsigned long duration;
bool settingMaxSpeeds = false;
int savedSpeeds[6] = {0, 0, 0, 0, 0, 0}; // Array to save the speeds

// Function Prototypes
void initializePWM();
void initializeADC();
int readPot(int adc_channel);
void setPWMDuty(int channel, int duty);
void accelerateToSpeed(int tracks[], int targetDuties[], int numTracks, int duration);
void decelerateToStop(int tracks[], int numTracks, int duration);
void handleTRE();
void handleMaxSpeedSetting();
void stopAllTrains();

void setup() {
  Serial.begin(115200);

  pinMode(sw1aPin, INPUT_PULLUP);
  pinMode(sw1bPin, INPUT_PULLUP);
  pinMode(sw3Pin, INPUT_PULLUP);

  initializePWM();
  initializeADC();

  Serial.println("Setup complete.");
}

void loop() {
  if (digitalRead(sw1aPin) == HIGH && digitalRead(sw1bPin) == HIGH) {
    if (trainRunning) {
      stopAllTrains();
    }
    settingMaxSpeeds = true;
    handleMaxSpeedSetting();
  } else if (digitalRead(sw3Pin) == LOW) {
    delay(50); // Debounce
    if (digitalRead(sw3Pin) == LOW) {
      handleTRE();
    }
  }
}

void initializePWM() {
  // Configure PWM timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_HIGH_SPEED_MODE,
    .duty_resolution  = (ledc_timer_bit_t)pwmResolution,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = PWM_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Initialize PWM channels for track loops
  for (int i = 0; i < 6; i++) {
    ledc_channel_config_t ledc_channel = {
      .gpio_num       = pwmPins[i],
      .speed_mode     = LEDC_HIGH_SPEED_MODE,
      .channel        = (ledc_channel_t)pwmChannels[i],
      .intr_type      = LEDC_INTR_DISABLE,
      .timer_sel      = LEDC_TIMER_0,
      .duty           = 0,
      .hpoint         = 0,
    };
    ledc_channel_config(&ledc_channel);
  }

  // Initialize PWM channel for MOSFET7
  ledc_channel_config_t ledc_channel = {
    .gpio_num       = mosfet7Pin,
    .speed_mode     = LEDC_HIGH_SPEED_MODE,
    .channel        = (ledc_channel_t)pwmChannels[6],
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = pwmMaxValue, // Set to 100%
    .hpoint         = 0,
  };
  ledc_channel_config(&ledc_channel);

  Serial.println("PWM initialized.");
}

void initializeADC() {
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

  for (int i = 0; i < 5; i++) {
    adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12, // Maximum attenuation
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, (adc_channel_t)adc1Channels[i], &config);
  }

  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN_DB_12, // Maximum attenuation
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  adc_oneshot_config_channel(adc2_handle, (adc_channel_t)adc2Channel, &config);

  Serial.println("ADC initialized.");
}

int readPot(int adc_channel) {
  int adcValue;
  if (adc_channel < 5) {
    adc_oneshot_read(adc1_handle, (adc_channel_t)adc1Channels[adc_channel], &adcValue);
  } else {
    adc_oneshot_read(adc2_handle, (adc_channel_t)adc2Channel, &adcValue);
  }
  int mappedValue = map(adcValue, 0, 4095, 0, pwmMaxValue);
  return mappedValue;
}

void setPWMDuty(int channel, int duty) {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)pwmChannels[channel], duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)pwmChannels[channel]);
}

void accelerateToSpeed(int tracks[], int targetDuties[], int numTracks, int duration) {
  int steps = 100;
  float stepDelay = (float)duration / steps;
  float increments[numTracks];

  for (int i = 0; i < numTracks; i++) {
    increments[i] = (float)targetDuties[i] / steps;
  }

  for (int step = 0; step <= steps; step++) {
    for (int i = 0; i < numTracks; i++) {
      setPWMDuty(tracks[i], (int)(step * increments[i]));
    }
    delay((int)stepDelay);
  }

  Serial.println("Acceleration complete.");
}

void decelerateToStop(int tracks[], int numTracks, int duration) {
  int steps = 100;
  float stepDelay = (float)duration / steps;
  float decrements[numTracks];
  float currentDuties[numTracks];

  for (int i = 0; i < numTracks; i++) {
    currentDuties[i] = (float)ledc_get_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)pwmChannels[tracks[i]]);
    decrements[i] = currentDuties[i] / steps;
  }

  for (int step = steps; step >= 0; step--) {
    for (int i = 0; i < numTracks; i++) {
      setPWMDuty(tracks[i], (int)(step * decrements[i]));
    }
    delay((int)stepDelay);
  }

  Serial.println("Deceleration complete.");
}

void handleTRE() {
  trainRunning = true;
  int activeTracks[5];
  int targetDuties[5];
  int index = 0;

  for (int i = 0; i < 6; i++) {
    if (i != idleTrack) {
      activeTracks[index] = i;
      targetDuties[index] = savedSpeeds[i];
      index++;
    }
  }

  Serial.print("Starting TRE with target duties: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(targetDuties[i]);
    if (i < 4) {
      Serial.print(", ");
    }
  }
  Serial.println();

  accelerateToSpeed(activeTracks, targetDuties, 5, ACCELERATION_TIME);

  idleTrack = (idleTrack + 1) % 6;

  if (digitalRead(sw1aPin) == LOW) {
    duration = RUN_TIME_SHORT * 1000;
  } else if (digitalRead(sw1bPin) == LOW) {
    duration = RUN_TIME_LONG * 1000;
  } else {
    return;
  }

  startTime = millis();
  while (millis() - startTime < duration) {
    for (int i = 0; i < 5; i++) {
      setPWMDuty(activeTracks[i], targetDuties[i]);
    }
    delay(100);
  }

  decelerateToStop(activeTracks, 5, ACCELERATION_TIME);

  trainRunning = false;
  Serial.println("TRE complete.");
}

void handleMaxSpeedSetting() {
  Serial.println("Entering speed setting mode...");
  while (digitalRead(sw1aPin) == HIGH && digitalRead(sw1bPin) == HIGH) {
    for (int i = 0; i < 6; i++) {
      int targetDuty = readPot(i);
      setPWMDuty(i, targetDuty);
    }
    delay(100);
  }

  for (int i = 0; i < 6; i++) {
    savedSpeeds[i] = ledc_get_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)pwmChannels[i]);
  }

  int tracks[6] = {0, 1, 2, 3, 4, 5};
  decelerateToStop(tracks, 6, ACCELERATION_TIME);

  settingMaxSpeeds = false;
  Serial.println("Exiting speed setting mode.");
}

void stopAllTrains() {
  Serial.println("Stopping all trains...");
  int tracks[6] = {0, 1, 2, 3, 4, 5};
  decelerateToStop(tracks, 6, ACCELERATION_TIME);
  trainRunning = false;
}