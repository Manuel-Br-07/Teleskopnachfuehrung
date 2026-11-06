#ifndef COMMAND_H
#define COMMAND_H
#include <stdint.h>

enum class CommandType : uint8_t { MoveRelative = 1, Stop = 2 };

// Nur feste, kopierbare Werte: FreeRTOS kopiert das Struct in die Queue.
struct Command {
  uint32_t id;
  CommandType type;
  int32_t steps; // wird für MoveRelative verwendet
};
#endif
