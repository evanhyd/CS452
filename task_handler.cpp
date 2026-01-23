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

TaskDescriptor* TaskScheduler::currentTask() { return currentTask_; }

TaskDescriptor& TaskScheduler::scheduleNextTask() { return queue.current(); }

void TaskScheduler::activate(TaskDescriptor& td) {
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
  *td = TaskDescriptor{
      .tid = globalTidCounter,
      .priority = priority,
      .parent = queue.empty() ? nullptr : &queue.current(),
      .stackMemory = ts,
      .nextReady = nullptr,
      .nextSend = nullptr,
      .runState = 0,
      .stackPointer = context,
  };
  queue.enque(*td);
  return globalTidCounter;
}

// Returns the task id of the calling task.
int MyTid() { return TaskScheduler::currentTask()->tid; }

// Returns the task id of the task that created the calling task.
// If the parent is dead, it may trigger undefined behavior such as launching the nuke (PLS DONT).
int MyParentTid() {
  if (TaskDescriptor* parent = TaskScheduler::currentTask()->parent) {
    return parent->tid;
  }
  return -1;
}

// Causes a task to pause executing.
// The task is moved to the end of its priority queue, and will resume executing when next scheduled.
void Yield() { queue.next(); }

// Causes a task to cease execution permanently. It is removed from all priority queues, send queues, receive queues and
// event queues. Resources owned by the task, primarily its memory and task descriptor, may be reclaimed.
void Exit() {
  queue.pop(); // TaskScheduler::currentTask() == queue.current()
  auto currTask = TaskScheduler::currentTask();
  taskStackAllocator.deallocate(currTask->stackMemory);
  taskDescriptorsAllocator.deallocate(currTask);
}

} // namespace syscall_handler
