#include "SerialHandler.h"
#include "Config.h"
#include "Command.h"
#include "Response.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>

static uint32_t nextCommandId = 1;

static bool submitLine(const char *line) {
  Command command{};
  command.id = nextCommandId++;
  long steps = 0;
  char extra = 0;

  if (sscanf(line, "MOVE %ld %c", &steps, &extra) == 1 && steps >= INT32_MIN && steps <= INT32_MAX) {
    command.type = CommandType::MoveRelative;
    command.steps = static_cast<int32_t>(steps);
  } else if (strcmp(line, "STOP") == 0) {
    command.type = CommandType::Stop;
    command.steps = 0;
  } else {
    Serial.println(F("ERR: Befehl erwartet: MOVE <steps> oder STOP"));
    return false;
  }

  // Timeout 0: bei voller Queue nicht blockieren; Fehler dem Nutzer melden.
  if (xQueueSend(commandQueue, &command, QUEUE_SEND_WAIT) != pdPASS) {
    Serial.println(F("ERR: Command Queue voll; Befehl verworfen"));
    return false;
  }
  return true;
}

void serialHandlerTask(void *parameter) {
  (void)parameter;
  char line[SERIAL_LINE_BUFFER_SIZE];
  size_t length = 0;
  bool overflow = false;

  for (;;) {
    while (Serial.available() > 0) {
      const char ch = static_cast<char>(Serial.read());
      if (ch == '\r') continue;
      if (ch == '\n') {
        if (!overflow && length > 0) {
          line[length] = '\0';
          submitLine(line);
        } else if (overflow) {
          Serial.println(F("ERR: Eingabezeile zu lang"));
        }
        length = 0;
        overflow = false;
      } else if (length + 1 < sizeof(line)) {
        line[length++] = ch;
      } else {
        overflow = true; // Rest der Zeile bis zum Zeilenende verwerfen
      }
    }

    // Antwortwarteschlange nicht-blockierend leeren, damit Ausgaben aktuell bleiben.
    Response response{};
    while (xQueueReceive(responseQueue, &response, 0) == pdPASS) {
      Serial.printf("RESP id=%lu type=%u code=%u value=%ld\n",
                    static_cast<unsigned long>(response.commandId),
                    static_cast<unsigned>(response.type),
                    static_cast<unsigned>(response.code),
                    static_cast<long>(response.value));
    }
    // Kleiner Yield, damit MotionPlanner und Systemtasks CPU-Zeit bekommen.
    vTaskDelay(TASK_YIELD_TICKS);
  }
}

