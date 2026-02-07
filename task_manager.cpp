#include "task_manager.h"
#include "debug.h"
#include "gic.h"
#include "syscall_task_handler.h"

namespace {

TaskDescriptor taskDescriptors[MAX_TASK_COUNT];
TaskStack taskStacks[MAX_TASK_COUNT];

MultiLevelQueue readyQueue{};
RoundRobinQueue eventBlockedQueue[2]{};
TaskDescriptor* currentTask = nullptr;

// idle task stuff
Tid idleTid;

size_t interruptIdToIndex(gic::InterruptEventId interruptId) {
  switch (interruptId) {
  case gic::InterruptEventId::TIMER1:
    return 0;
  case gic::InterruptEventId::TIMER3:
    return 1;
  default:
    logError("unknown event type");
  }
}

} // namespace

TaskDescriptor* Tid::descriptor() const { return &taskDescriptors[index()]; }

TaskStack* Tid::stack() const { return &taskStacks[index()]; }

Tid TidAllocator::allocate() {
  size_t index = free[--top];
  return Tid{generations[index], index};
}

void TidAllocator::deallocate(Tid tid) {
  size_t index = tid.index();
  if (generations[index] != tid.generation()) {
    logError("Double free in TidAllocator");
  }
  ++generations[index];
  free[top++] = index;
}

bool TidAllocator::isAlive(Tid tid) const { return generations[tid.index()] == tid.generation(); }

TaskDescriptor* TidAllocator::getTaskDescriptor(int rawTid) const {
  Tid tid = Tid::fromRaw(rawTid);
  if (!isAlive(tid)) {
    return nullptr;
  }
  return tid.descriptor();
}

TaskDescriptor* TaskScheduler::getCurrentTask() { return currentTask; }

TaskDescriptor* TaskScheduler::getNextScheduledTask() { return readyQueue.current(); }

void TaskScheduler::enqueReadyTask(TaskDescriptor& td) { readyQueue.enque(td); }

void TaskScheduler::moveReadyTaskToEnd(TaskDescriptor& td) { readyQueue.moveToEnd(td); }

void TaskScheduler::removeReadyTask(TaskDescriptor& td) { readyQueue.remove(td); }

void TaskScheduler::enqueEventBlockedTask(gic::InterruptEventId eventId, TaskDescriptor& td) {
  size_t index = interruptIdToIndex(eventId);
  eventBlockedQueue[index].enque(td);
}

void TaskScheduler::notifyAllEventBlockedTasks(gic::InterruptEventId eventId, int eventValue) {
  size_t index = interruptIdToIndex(eventId);
  while (!eventBlockedQueue[index].empty()) {
    TaskDescriptor* task = eventBlockedQueue[index].pop();
    if (!task) {
      logError("detected null task in the event queue");
    }
    task->setRetValue(eventValue);
    task->runState = RunState::READY;
    enqueReadyTask(*task);
  }
}

extern "C" [[noreturn]] void switchTask(void* sp);

void TaskScheduler::activateTask(TaskDescriptor& td) {
  currentTask = &td;
  switchTask(td.stackPointer);
}

void createIdleTask() {
  int tid = syscall_handler::Create(Priority::LOWEST, []() {
    while (true) {
      asm("wfi");
    }
  });
  if (tid < 0) {
    logError("failed to create idle task");
  }
  idleTid = Tid::fromRaw(tid);
}
