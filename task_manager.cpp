#include "task_manager.h"
#include "debug.h"

namespace {

TaskDescriptor taskDescriptors[MAX_TASK_COUNT];
TaskStack taskStacks[MAX_TASK_COUNT];

MultiLevelQueue readyQueue{};
TaskDescriptor* currentTask = nullptr;

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

void TaskScheduler::enqueTask(TaskDescriptor& td) { readyQueue.enque(td); }

void TaskScheduler::moveTaskToEnd(TaskDescriptor& td) { readyQueue.moveToEnd(td); }

void TaskScheduler::removeTask(TaskDescriptor& td) { readyQueue.remove(td); }

extern "C" [[noreturn]] void switchTask(void* sp);

void TaskScheduler::activateTask(TaskDescriptor& td) {
  currentTask = &td;
  switchTask(td.stackPointer);
}
