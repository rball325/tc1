#include <Arduino.h>

void showResetReason(void)
{
  // Print the reason for the last reset
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  switch (reason) {
    case ESP_RST_POWERON:
      Serial.println("Power on reset");
      break;
    case ESP_RST_EXT:
      Serial.println("External reset");
      break;
    case ESP_RST_SW:
      Serial.println("Software reset");
      break;
    case ESP_RST_PANIC:
      Serial.println("Exception/panic reset");
      break;
    case ESP_RST_INT_WDT:
      Serial.println("Watchdog reset (interrupt)");
      break;
    case ESP_RST_TASK_WDT:
      Serial.println("Watchdog reset (task)");
      break;
    case ESP_RST_WDT:
      Serial.println("Watchdog reset");
      break;
    case ESP_RST_DEEPSLEEP:
      Serial.println("Deep sleep reset");
      break;
    case ESP_RST_BROWNOUT:
      Serial.println("Brownout reset");
      break;
    case ESP_RST_SDIO:
      Serial.println("SDIO reset");
      break;
    default:
      Serial.println("Unknown reset reason");
      break;
  }
}

const int debounceDelay = 100; // Debounce delay in milliseconds
const int MAX_SWITCHES = 10;

struct switch_s {
  int inputPin;
  int lastButtonState;  // Previous state of the button
  int buttonState;      // Current state of the button
  unsigned long lastDebounceTime; // Time the button state was last changed
};

static switch_s switches[MAX_SWITCHES] = {0};

void switch_setup(int index, int inputPin) 
{
  pinMode(inputPin, INPUT_PULLUP);  // Set GPIO as an input with a pullup 
  switches[index].inputPin = inputPin;
}

void switch_update(int index) 
{
  // Read the current state of the button
  int reading = digitalRead(switches[index].inputPin);

  // Check if the button state has changed
  if (reading != switches[index].lastButtonState) {
    // Reset the debouncing timer
    switches[index].lastDebounceTime = millis();
  }
  
  // If the state has been stable for longer than the debounce delay
  if ((millis() - switches[index].lastDebounceTime) > debounceDelay) {
    // Update the button state
    if (reading != switches[index].buttonState) {
      switches[index].buttonState = reading;
      
      // Print the button state to the Serial Monitor
      Serial.print("Button ");
      Serial.print(index);
      if (switches[index].buttonState == LOW) {
        Serial.println(" pressed");
      } else {
        Serial.println(" released");
      }
    }
  }
  
  // Save the reading for the next loop
  switches[index].lastButtonState = reading;
}

bool switch_state(int index)
{
  return switches[index].buttonState;
}