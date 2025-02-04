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
