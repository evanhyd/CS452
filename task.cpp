#include "task.h"
#include "slab_allocator.h"
#include "task_queue.h"
#include <cstddef>
#include <iterator>
#include <limits>
#include <utility>

namespace {
struct TaskStack {
  static constexpr size_t TASK_STACK_SIZE = 1 << 20;
  alignas(16) std::byte data[TASK_STACK_SIZE];
  void* top() { return std::end(data); }
};
SlabAllocator<TaskDescriptor, 128> taskAllocator{};
SlabAllocator<TaskStack, 128> taskStackAllocator{};

MultiLevelQueue<RoundRobinQueue> queue{};
int globalTidCounter = 0;
} // namespace

TaskDescriptor& TaskScheduler::scheduleNextTask() {
  queue.next();
  return queue.current();
}

void TaskScheduler::activate(TaskDescriptor& td) {
  // TODO: implement this properly.
  if (td.entryFunction) {
    std::exchange(td.entryFunction, nullptr)();
  }
}

extern "C" {
// Return the positive integer task id of the newly created task.
// -1 invalid priority.
// -2 kernel is out of task descriptors.
int Create(int priority, void (*function)()) {

  // Check priority.
  if (priority >= Priority::COUNT) {
    return -1;
  }

  // Check if out of task descriptors.
  if (taskAllocator.full()) {
    return -2;
  }

  // Check tid overflow
  if (globalTidCounter == std::numeric_limits<decltype(globalTidCounter)>::max()) {
    return -2;
  }
  ++globalTidCounter;

  // Allocate and register a new task.
  TaskDescriptor* td = taskAllocator.allocate();
  TaskStack* ts = taskStackAllocator.allocate();

  *td = TaskDescriptor{
      .tid = globalTidCounter,
      .priority = priority,
      .parent = &queue.current(),
      .nextReady = nullptr,
      .nextSend = nullptr,
      .entryFunction = function,
      .runState = nullptr,       // ???
      .stackPointer = ts->top(), // ???
  };

  queue.enque(*td);
  return globalTidCounter;
}

// Returns the task id of the calling task.
int MyTid() { return queue.current().tid; }

// Returns the task id of the task that created the calling task.
// If the parent is dead, it may trigger undefined behavior such as launching the nuke (PLS DONT).
int MyParentTid() { return queue.current().parent->tid; }

// Causes a task to pause executing.
// The task is moved to the end of its priority queue, and will resume executing when next scheduled.
void Yield() {
  // TDOO: add implementation.
}

// Causes a task to cease execution permanently. It is removed from all priority queues, send queues, receive queues and
// event queues. Resources owned by the task, primarily its memory and task descriptor, may be reclaimed.
void Exit() {
  // TDOO: add implementation.
}
}
