#pragma once

#include "slab_allocator.h"

#include <cstddef>

// A placeholder class to define the size of the task stack frame.
// Configure TASK_STACK_SIZE to change the task stack size.
struct TaskStack {
  static constexpr size_t TASK_STACK_SIZE = 1 << 20;
  alignas(16) std::byte data[TASK_STACK_SIZE];
  void* top() { return data + TASK_STACK_SIZE; }
  void* offsetFromTop(size_t bytes) { return data + TASK_STACK_SIZE - bytes; }
};

// Allocated in kernel memory during kernel initialization.
// Every existing task has a TD allocated to it.
struct TaskDescriptor {
  int tid; // task identifier
  int priority;
  TaskDescriptor* parent;
  TaskStack* stackMemory;
  TaskDescriptor* nextReady; // next task in the task's ready queue
  TaskDescriptor* nextSend;  // next task on the task's send queuee

  int runState;       // ready, suspend etc
  void* stackPointer; // sp
};

// A singleton class that schedules the tasks.
// Internally, it uses a multi-level round robin queue.
struct TaskScheduler {
  TaskScheduler() = delete;
  static TaskDescriptor* getCurrentTask();
  static TaskDescriptor* getNextScheduledTask();
  static void scheduleNextTask();
  [[noreturn]] static void activateTask(TaskDescriptor& td);
};

// A palceholder class to define the saved context of each user task.
struct StackContext {
  uint64_t elr_el1;
  uint64_t spsr_el1;
  uint64_t x30;
  uint64_t esr_el1;
  uint64_t x28, x29;
  uint64_t x26, x27;
  uint64_t x24, x25;
  uint64_t x22, x23;
  uint64_t x20, x21;
  uint64_t x18, x19;
  uint64_t x16, x17;
  uint64_t x14, x15;
  uint64_t x12, x13;
  uint64_t x10, x11;
  uint64_t x8, x9;
  uint64_t x6, x7;
  uint64_t x4, x5;
  uint64_t x2, x3;
  uint64_t x0, x1;
};
static_assert(sizeof(StackContext) % 16 == 0, "sp must aligned to 16");

namespace syscall_handler {

int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();

} // namespace syscall_handler
