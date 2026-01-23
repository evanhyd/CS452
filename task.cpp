#include "task.h"
#include "slab_allocator.h"
#include "task_queue.h"
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>

namespace {

struct TaskStack {
  static constexpr size_t TASK_STACK_SIZE = 1 << 20;
  alignas(16) std::byte data[TASK_STACK_SIZE];
  void* offsetFromTop(size_t bytes) { return std::end(data) - bytes; }
};

struct StackContext {
  uint64_t x[31];  // x0 - x30
  uint64_t xzr;    // padding
  uint64_t pstate; // SPSR_EL1
  uint64_t pc;     // ELR_EL1
  StackContext() = delete;
};
static_assert(sizeof(StackContext) % 16 == 0, "sp must aligned to 16");

SlabAllocator<TaskDescriptor, 128> taskDescriptorsAllocator{};
SlabAllocator<TaskStack, 128> taskStackAllocator{};
MultiLevelQueue<RoundRobinQueue> queue{};
int globalTidCounter = 0;

extern "C" void switchTask(uint64_t sp);

// A wrapper function to the actual task entry.
// This automatically frees up the task.
void taskEntryWrapper(TaskEntry entry) {
  entry();
  Exit();
}

} // namespace

TaskDescriptor& TaskScheduler::scheduleNextTask() {
  queue.next();
  return queue.current();
}

void TaskScheduler::activate(TaskDescriptor& td) { switchTask(td.stackPointer); }

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
  memset(ts->offsetFromTop(sizeof(StackContext)), 0, sizeof(StackContext));
  *(uint64_t*)(ts->offsetFromTop(sizeof(StackContext))) = uint64_t(taskEntryWrapper);
  *(uint64_t*)(ts->offsetFromTop(sizeof(uintptr_t))) = uint64_t(function);

  // Allocate a new task descriptor.
  TaskDescriptor* td = taskDescriptorsAllocator.allocate();
  *td = TaskDescriptor{
      .tid = globalTidCounter,
      .priority = priority,
      .parent = nullptr, // &queue.current() hangs the program, as there's no task in the queue currently. we need to
                         // discuss how we should define the parent of the first task.
      .nextReady = nullptr,
      .nextSend = nullptr,
      .runState = 0,
      .stackPointer = uintptr_t(ts->offsetFromTop(sizeof(StackContext))),
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
