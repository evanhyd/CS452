#include "task_handler.h"

#include "task_manager.h"
#include <cstdint>

namespace syscall_handler {

// Return the positive integer task id of the newly created task.
// -1 invalid priority.
// -2 kernel is out of task descriptors.
int Create(int priority, void (*function)()) {

  // Check priority.
  if (priority >= Priority::COUNT) {
    return -1;
  }

  // Check if out of TIDs.
  if (tidAllocator.full()) {
    return -2;
  }
  Tid tid = tidAllocator.allocate();

  // Initialize task stack with initial context.
  // Initialize all 32 registers: x0-x30, Pstate, ELR.
  // Set up entry to the task wrapper.
  TaskStack* ts = tid.stack();
  StackContext* context = new (ts->offsetFromTop(sizeof(StackContext))) StackContext{};
  context->elr_el1 = reinterpret_cast<uint64_t>(function);
  context->x30 = reinterpret_cast<uint64_t>(&::Exit);

  // Initialize task descriptor.
  TaskDescriptor* td = tid.descriptor();
  TaskDescriptor* currTask = TaskScheduler::getCurrentTask();
  *td = TaskDescriptor{.tid = tid,
                       .priority = priority,
                       .parentTid = currTask ? currTask->tid : Tid::invalid(),
                       .stackMemory = ts,
                       .next = nullptr,
                       .stackPointer = context,
                       .runState = RunState::READY,
                       .messageControlBlock = {},
                       .sendWaitQueue = {}};
  TaskScheduler::enqueTask(*td);
  return tid.raw();
}

// Returns the task id of the calling task.
int MyTid() { return TaskScheduler::getCurrentTask()->tid.raw(); }

// Returns the task id of the task that created the calling task.
// If the task has no parent (i.e. the initial task created by the kernel), return -1.
// If the parent is dead, it still returns the original parent tid.
int MyParentTid() { return TaskScheduler::getCurrentTask()->parentTid.raw(); }

// Causes a task to pause executing.
// The task is moved to the end of its priority queue, and will resume executing when next scheduled.
void Yield() { TaskScheduler::moveTaskToEnd(*TaskScheduler::getCurrentTask()); }

// Causes a task to cease execution permanently. It is removed from all priority queues, send queues, receive queues and
// event queues. Resources owned by the task, primarily its memory and task descriptor, may be reclaimed.
void Exit() {
  auto currTask = TaskScheduler::getCurrentTask();
  TaskScheduler::removeTask(*currTask);
  tidAllocator.deallocate(currTask->tid);
}

} // namespace syscall_handler
