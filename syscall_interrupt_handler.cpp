#include "syscall_interrupt_handler.h"
#include "debug.h"
#include "gic.h"
#include "gpio.h"
#include "mcp2515.h"
#include "task_manager.h"
#include "uart.h"

namespace syscall_handler {

static constexpr int RET_PLACEHOLDER = -69;

// Blocks until the event identified by eventid occurs then returns with event-specific data, if any.
// Return Value
// >=0	event-specific data, in the form of a positive integer.
// -1	invalid event.
int AwaitEvent(int eventId) {
  auto currTask = TaskScheduler::getCurrentTask();
  if (!currTask) {
    logError("dereference null task");
  }
  if (currTask->runState != RunState::Ready) {
    logError("task is not ready");
  }

  if (!gic::isValidEventId(eventId)) {
    return -1;
  }

  ::EventId event = static_cast<::EventId>(eventId);
  switch (event) {
  case ::EventId::UartRx:
    Uart::enableRxInterrupt();
    break;
  case ::EventId::UartTx:
    Uart::enableTxInterrupt();
    break;
  case ::EventId::CanIO:
    gpio::set_pin_low_detect(17, true);
    break;
  default:
    break;
  }

  currTask->runState = RunState::EventBlocked;
  TaskScheduler::removeReadyTask(*currTask);
  TaskScheduler::enqueEventBlockedTask(event, *currTask);

  return RET_PLACEHOLDER;
}
} // namespace syscall_handler
