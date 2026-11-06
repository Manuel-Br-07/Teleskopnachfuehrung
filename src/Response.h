#ifndef RESPONSE_H
#define RESPONSE_H
#include <stdint.h>

enum class ResponseType : uint8_t { Accepted = 1, Status = 2, Error = 3 };
enum class ResponseCode : uint8_t { None = 0, Started = 1, Stopped = 2, InvalidCommand = 3, Busy = 4 };

struct Response {
  uint32_t commandId;
  ResponseType type;
  ResponseCode code;
  int32_t value; // z.B. angeforderte Schritte oder Statuswert
};
#endif
