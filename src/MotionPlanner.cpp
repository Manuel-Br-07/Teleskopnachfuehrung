#include "MotionPlanner.h"
#include "Config.h"
#include "Command.h"
#include "Response.h"

static void sendResponse(uint32_t id, ResponseType type, ResponseCode code, int32_t value) {
  const Response response{id, type, code, value};
  // Queue voll: Antwort kann nicht zugestellt werden. Loggen; niemals auf Serial zugreifen.
  if (xQueueSend(responseQueue, &response, QUEUE_SEND_WAIT) != pdPASS) {
    // Bewusst still: MotionPlanner besitzt Serial nicht. In produktivem Code Zähler/Diagnose ergänzen.
  }
}

void motionPlannerTask(void *parameter) {
  (void)parameter;
  Command command{};
  for (;;) {
    // Blockiert effizient bis zum nächsten Kommando, gibt dabei die CPU frei.
    if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdPASS) {
      switch (command.type) {
        case CommandType::MoveRelative:
          // Beispiel-Skelett: hier Bewegungsprofil/Stepper ansteuern.
          sendResponse(command.id, ResponseType::Accepted, ResponseCode::Started, command.steps);
          break;
        case CommandType::Stop:
          // Beispiel-Skelett: hier laufende Bewegung sicher stoppen.
          sendResponse(command.id, ResponseType::Status, ResponseCode::Stopped, 0);
          break;
        default:
          sendResponse(command.id, ResponseType::Error, ResponseCode::InvalidCommand, 0);
          break;
      }
    }
  }
}
