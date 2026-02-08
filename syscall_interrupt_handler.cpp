#include "syscall_interrupt_handler.h"
#include "debug.h"
#include "gic.h"
#include "task_manager.h"
#include "task_queue.h"

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
  if (currTask->runState != RunState::READY) {
    logError("task is not ready");
  }

  if (!gic::isValidInterruptEventId(eventId)) {
    return -1;
  }

  currTask->runState = RunState::EVENT_BLOCKED;
  TaskScheduler::removeReadyTask(*currTask);
  TaskScheduler::enqueEventBlockedTask(static_cast<gic::InterruptEventId>(eventId), *currTask);

  return RET_PLACEHOLDER;
}
} // namespace syscall_handler
