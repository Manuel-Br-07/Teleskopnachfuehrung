#include "Config.h"
#include "SerialHandler.h"
#include "MotionPlanner.h"
#include "Command.h"
#include "Response.h"

QueueHandle_t commandQueue = nullptr;
QueueHandle_t responseQueue = nullptr;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(Command));
  responseQueue = xQueueCreate(RESPONSE_QUEUE_LENGTH, sizeof(Response));
  if (commandQueue == nullptr || responseQueue == nullptr) {
    Serial.println(F("FATAL: Queue-Erstellung fehlgeschlagen"));
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // ESP32 port: Stackgröße in Bytes. Core-Affinity und Priorität zentral konfiguriert.
  const BaseType_t serialResult = xTaskCreatePinnedToCore(
      serialHandlerTask, "SerialHandler", SERIAL_TASK_STACK_BYTES, nullptr,
      SERIAL_TASK_PRIORITY, nullptr, SERIAL_TASK_CORE);
  const BaseType_t motionResult = xTaskCreatePinnedToCore(
      motionPlannerTask, "MotionPlanner", MOTION_TASK_STACK_BYTES, nullptr,
      MOTION_TASK_PRIORITY, nullptr, MOTION_TASK_CORE);

  if (serialResult != pdPASS || motionResult != pdPASS) {
    Serial.println(F("FATAL: Task-Erstellung fehlgeschlagen"));
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void loop() {
  // Die Anwendung läuft in den beiden FreeRTOS-Tasks; Arduino loop gibt CPU frei.
  vTaskDelay(pdMS_TO_TICKS(1000));
}

