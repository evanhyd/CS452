#include "task_manager.h"
#include "debug.h"
#include "fmt.h"
#include "gic.h"
#include "syscall_task_handler.h"
#include "timer.h"
#include "uart.h"

namespace {

TaskDescriptor taskDescriptors[MAX_TASK_COUNT];
TaskStack taskStacks[MAX_TASK_COUNT];

MultiLevelQueue readyQueue{};
RoundRobinQueue eventBlockedQueue[2]{};
TaskDescriptor* currentTask = nullptr;

// idle task stuff
Tid idleTid;
timer::Time intervalStart;
timer::Time idlePart;
timer::Time lastSwitchTime;

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
  using namespace timer::literals;

  auto now = timer::system_timer.now();
  if (currentTask) {
    if (currentTask->tid == idleTid) {
      idlePart += now - lastSwitchTime;
    }
  } else {
    intervalStart = now;
  }

  auto delta = now - intervalStart;
  if (delta >= 500_ms) {
    auto idlePerMille = static_cast<uint64_t>(idlePart.micros()) * 1000 / delta.micros();
    auto idlePercent = idlePerMille / 10;
    auto idleFraction = idlePerMille % 10;

    // TODO: use ansi to print in place somewhere?
    char buf[64];
    char* end = kit::strAppend(buf, "Idle: ");
    if (idlePercent == 100) {
      end = kit::strAppend(end, "100.");
    } else {
      if (idlePercent < 10) {
        end = kit::strAppend(end, ' ');
      } else {
        end = kit::strAppend(end, static_cast<char>('0' + idlePercent / 10));
      }
      end = kit::strAppend(end, static_cast<char>('0' + idlePercent % 10));
      end = kit::strAppend(end, '.');
      end = kit::strAppend(end, static_cast<char>('0' + idleFraction));
    }
    end = kit::strAppend(end, "%\r\n");
    Uart::syncPrint(Uart::CONSOLE, buf);

    intervalStart = now;
    idlePart = 0_us;
  }

  currentTask = &td;
  lastSwitchTime = now;
  switchTask(td.stackPointer);
}

void createIdleTask() {
  int tid = syscall_handler::Create(4, []() {
    while (true) {
      asm("wfi");
    }
  });
  if (tid < 0) {
    logError("failed to create idle task");
  }
  idleTid = Tid::fromRaw(tid);
}
