#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Zentrale Parameter. Queue-Längen, Prioritäten und Stacks sind hier anpassbar.
constexpr UBaseType_t COMMAND_QUEUE_LENGTH = 8;
constexpr UBaseType_t RESPONSE_QUEUE_LENGTH = 8;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint32_t SERIAL_TASK_STACK_BYTES = 4096; // Parser + Serial-Ausgaben
constexpr uint32_t MOTION_TASK_STACK_BYTES = 4096; // später Platz für Bewegungsplanung
constexpr UBaseType_t SERIAL_TASK_PRIORITY = 2;
constexpr UBaseType_t MOTION_TASK_PRIORITY = 2;
constexpr BaseType_t SERIAL_TASK_CORE = 0;
constexpr BaseType_t MOTION_TASK_CORE = 1;
constexpr TickType_t QUEUE_SEND_WAIT = 0; // Tasks bleiben bei vollen Queues reaktionsfähig
constexpr TickType_t TASK_YIELD_TICKS = 1;
constexpr size_t SERIAL_LINE_BUFFER_SIZE = 80;

extern QueueHandle_t commandQueue;
extern QueueHandle_t responseQueue;
#endif
