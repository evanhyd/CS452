#include "clock_server.h"
#include "debug.h"
#include "gic.h"
#include "syscalls.h"

namespace clock_server {

enum class ClockServerMessageType : int { TIME, DELAY, DELAY_UNTIL };

struct TimeMessage {};

struct DelayMessage {
  int ticks;
};

struct DelayUntilMessage {
  int ticks;
};

struct ClockServerMessage {
  ClockServerMessageType type;
  union {
    TimeMessage timeMessage;
    DelayMessage delayMessage;
    DelayUntilMessage delayUntilMessage;
  };
};

// Returns the number of ticks since the clock server was created and initialized.
// Return Value
// >=0	time in ticks since the clock server initialized.
// -1	tid is not a valid clock server task.
int Time(int tid) {
  ClockServerMessage msg{.type = ClockServerMessageType::TIME, .timeMessage = TimeMessage{}};
  int value;
  ::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(ClockServerMessage), reinterpret_cast<char*>(&value),
         sizeof(value));
  return value;
}

// Blocks the caller until at least until the specified number of ticks has elapsed since the call to Delay, according
// to the time server identified by tid. Return Value
// >=0	success. The current time returned (as in Time())
// -1	tid is not reachable or is not a time server
// -2	negative delay.
int Delay(int tid, int ticks) {
  ClockServerMessage msg{.type = ClockServerMessageType::DELAY, .delayMessage = DelayMessage{ticks}};
  int value;
  ::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(ClockServerMessage), reinterpret_cast<char*>(&value),
         sizeof(value));
  return value;
}

// Blocks the caller at least until the specified number of ticks has elapsed since the initialization of time server
// tid.
// Return Value
// >=0	success. The current time returned (as in Time())
// -1	tid is not reachable or is not a time server
// -2	negative delay.
int DelayUntil(int tid, int ticks) {
  ClockServerMessage msg{.type = ClockServerMessageType::DELAY_UNTIL, .delayUntilMessage = DelayUntilMessage{ticks}};
  int value;
  ::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(ClockServerMessage), reinterpret_cast<char*>(&value),
         sizeof(value));
  return value;
}

void clockServerTask() {
  const int initializedTick = ::AwaitEvent(static_cast<int>(gic::InterruptEventId::TIMER1));
  if (initializedTick == -1) {
    logError("invalid timer event data");
  }

  for (;;) {
    int tid;
    ClockServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(ClockServerMessage));

    // Process the query.
    switch (msg.type) {
    case ClockServerMessageType::TIME:
      // create clock notifier task
      // awaitEvent()
      break;
    case ClockServerMessageType::DELAY:
      break;
    case ClockServerMessageType::DELAY_UNTIL:
      break;
    default:
      break;
    }
  }
}
} // namespace clock_server