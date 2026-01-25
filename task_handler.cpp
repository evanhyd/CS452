#include "task_handler.h"

#include "slab_allocator.h"
#include "task.h"
#include "task_queue.h"
#include <cstdint>
#include <limits>
#include <new>

namespace {

SlabAllocator<TaskDescriptor, 128> taskDescriptorsAllocator{};
SlabAllocator<TaskStack, 128> taskStackAllocator{};

MultiLevelQueue<RoundRobinQueue> queue{};
TaskDescriptor* currentTask_ = nullptr;
int globalTidCounter = 0;

} // namespace

extern "C" [[noreturn]] void switchTask(void* sp);

// Return the task descriptor of the currently running task, or nullptr if there isn't any.
TaskDescriptor* TaskScheduler::getCurrentTask() { return currentTask_; }

// Return the task descriptor of the next scheduled task, or nullptr if there isn't any.
TaskDescriptor* TaskScheduler::getNextScheduledTask() { return queue.current(); }

// Move the given task to the end of its priority queue.
void TaskScheduler::moveTaskToEnd(TaskDescriptor& td) { queue.moveToEnd(td); }

// Remove the given task from its priority queue.
void TaskScheduler::removeTask(TaskDescriptor& td) { queue.remove(td); }

// Context switch to the task denoted by its task descriptor.
void TaskScheduler::activateTask(TaskDescriptor& td) {
  currentTask_ = &td;
  switchTask(td.stackPointer);
}

namespace syscall_handler {

// Return the positive integer task id of the newly created task.
// -1 invalid priority.
// -2 kernel is out of task descriptors.
int Create(int priority, void (*function)()) {

  // Check priority.
  if (priority >= Priority::COUNT) {
    return -1;
  }

  // Check if out of task descriptors.
  if (taskDescriptorsAllocator.full()) {
    return -2;
  }

  // Check tid overflow
  if (globalTidCounter == std::numeric_limits<decltype(globalTidCounter)>::max()) {
    return -2;
  }
  ++globalTidCounter;

  // Allocate a new task stack.
  // Initialize all 32 registers: x0-x30, Pstate, ELR.
  // Set up entry to the task wrapper.
  TaskStack* ts = taskStackAllocator.allocate();
  StackContext* context = std::launder(reinterpret_cast<StackContext*>(ts->offsetFromTop(sizeof(StackContext))));
  memset(context, 0, sizeof(StackContext));
  context->elr_el1 = reinterpret_cast<uint64_t>(function);
  context->x30 = reinterpret_cast<uint64_t>(&::Exit);

  // Allocate a new task descriptor.
  TaskDescriptor* td = taskDescriptorsAllocator.allocate();
  TaskDescriptor* currTask = TaskScheduler::getCurrentTask();
  *td = TaskDescriptor{
      .tid = globalTidCounter,
      .priority = priority,
      .parentTid = currTask ? currTask->tid : -1,
      .stackMemory = ts,
      .nextReady = nullptr,
      .stackPointer = context,
  };
  queue.enque(*td);
  return globalTidCounter;
}

// Returns the task id of the calling task.
int MyTid() { return TaskScheduler::getCurrentTask()->tid; }

// Returns the task id of the task that created the calling task.
// If the task has no parent (i.e. the initial task created by the kernel), return -1.
// If the parent is dead, it still returns the original parent tid.
int MyParentTid() { return TaskScheduler::getCurrentTask()->parentTid; }

// Causes a task to pause executing.
// The task is moved to the end of its priority queue, and will resume executing when next scheduled.
void Yield() { TaskScheduler::moveTaskToEnd(*TaskScheduler::getCurrentTask()); }

// Causes a task to cease execution permanently. It is removed from all priority queues, send queues, receive queues and
// event queues. Resources owned by the task, primarily its memory and task descriptor, may be reclaimed.
void Exit() {
  auto currTask = TaskScheduler::getCurrentTask();
  TaskScheduler::removeTask(*currTask);
  taskStackAllocator.deallocate(currTask->stackMemory);
  taskDescriptorsAllocator.deallocate(currTask);
}

} // namespace syscall_handler
