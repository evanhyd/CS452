#include "clock_server.h"
#include "debug.h"
#include "static_priority_queue.h"
#include "syscalls.h"

namespace {

enum class ClockServerMessageType : int { TIME, DELAY, DELAY_UNTIL, NOTIFIER_UPDATE };

struct TimeMessage {};

struct DelayMessage {
  int ticks;
};

struct DelayUntilMessage {
  int ticks;
};

struct NotifierUpdateMessage {
  int ticks;
};

struct ClockServerMessage {
  ClockServerMessageType type;
  union {
    TimeMessage timeMessage;
    DelayMessage delayMessage;
    DelayUntilMessage delayUntilMessage;
    NotifierUpdateMessage notifierUpdateMessage;
  };
};

struct DelayRequest {
  int wakeTime;
  int tid;
  friend bool operator<(const DelayRequest& lhs, const DelayRequest& rhs) { return lhs.wakeTime < rhs.wakeTime; }
};

void clockNotifierTask() {
  int serverTid = ::MyParentTid();
  for (;;) {
    int ticks = ::AwaitEvent(::EventId::TIMER1);
    ClockServerMessage msg = {.type = ClockServerMessageType::NOTIFIER_UPDATE,
                              .notifierUpdateMessage = NotifierUpdateMessage{ticks}};
    char devnull;
    ::Send(serverTid, reinterpret_cast<const char*>(&msg), sizeof(ClockServerMessage), &devnull, 0);
  }
}

} // namespace

void clock_server::clockServerTask() {
  if (::RegisterAs(CLOCK_SERVER_NAME) < 0) {
    logError("clock server failed to register itself to name server");
  }

  int ticks = 0;
  StaticPriorityQueue<DelayRequest, 128> delayQueue;
  [[maybe_unused]] int notifierTid = ::Create(0, clockNotifierTask);

  for (;;) {
    int tid;
    ClockServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(ClockServerMessage));

    // Process the query.
    switch (msg.type) {
    case ClockServerMessageType::TIME:
      ::Reply(tid, reinterpret_cast<const char*>(&ticks), sizeof(ticks));
      break;
    case ClockServerMessageType::DELAY:
      if (delayQueue.full()) {
        logError("clock server delay queue is full?!");
      }
      delayQueue.push(DelayRequest{.wakeTime = ticks + msg.delayMessage.ticks, .tid = tid});
      break;
    case ClockServerMessageType::DELAY_UNTIL:
      if (delayQueue.full()) {
        logError("clock server delay queue is full?!");
      }
      delayQueue.push(DelayRequest{.wakeTime = msg.delayUntilMessage.ticks, .tid = tid});
      break;
    case ClockServerMessageType::NOTIFIER_UPDATE:
      ++ticks;
      while (!delayQueue.empty() && delayQueue.top().wakeTime <= ticks) {
        ::Reply(delayQueue.top().tid, reinterpret_cast<const char*>(&ticks), sizeof(ticks));
        delayQueue.pop();
      }
      {
        const char dummy{};
        ::Reply(tid, &dummy, 0);
      }
      break;
    default:
      break;
    }
  }
}

// Returns the number of ticks since the clock server was created and initialized.
// Return Value
// >=0	time in ticks since the clock server initialized.
// -1	tid is not a valid clock server task.
extern "C" int Time(int tid) {
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
extern "C" int Delay(int tid, int ticks) {
  if (ticks < 0) {
    return -2;
  }
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
extern "C" int DelayUntil(int tid, int ticks) {
  ClockServerMessage msg{.type = ClockServerMessageType::DELAY_UNTIL, .delayUntilMessage = DelayUntilMessage{ticks}};
  int value;
  ::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(ClockServerMessage), reinterpret_cast<char*>(&value),
         sizeof(value));
  return value;
}
