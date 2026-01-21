#include "task.h"
#include "task_queue.h"

namespace {
SlabAllocator<TaskDescriptor, 32> taskAllocator;
MultiLevelQueue<RoundRobinQueue> queue{};
int globalTidCounter = 0;
} // namespace

extern "C" {

// Return the positive integer task id of the newly created task.
// -1 invalid priority.
// -2 kernel is out of task descriptors.
int Create(int priority, void (*function)()) {

  // Check priority.
  if (priority >= Priority::COUNT) {
    return -1;
  }

  // Check tid overflow.
  ++globalTidCounter;
  if (globalTidCounter == 0) {
    return -2;
  }

  // Allocate and register a new task.
  TaskDescriptor* td = taskAllocator.allocate();
  td->tid = globalTidCounter;
  td->priority = priority;
  return globalTidCounter;
}

// Returns the task id of the calling task.
int MyTid() { return queue.current().tid; }

// Returns the task id of the task that created the calling task.
// If the parent is dead, it may trigger undefined behavior such as launching the nuke (PLS DONT).
int MyParentTid() {
  // TDOO: add implementation.
  return 0;
}

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
