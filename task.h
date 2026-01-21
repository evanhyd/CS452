#pragma once
#include "slab_allocator.h"

enum Priority : int { HIGH, MEDIUM, LOW, COUNT };

// Allocated in kernel memory during kernel initialization.
// Every existing task has a TD allocated to it.
struct TaskDescriptor {
  int tid; // task identifier
  int priority;
  TaskDescriptor* parent;
  TaskDescriptor* nextReady; // next task in the task's ready queue
  TaskDescriptor* nextSend;  // next task on the task's send queue
  void (*entryFunction)();
  void* runState;               // unknown type
  void* stackPointer = nullptr; // unknown type
  // TODO: the task's return value, and the task's SPSR (either in TD or stack)
};

struct TaskScheduler {
  TaskScheduler() = delete;

  static TaskDescriptor& scheduleNextTask();
  static void activate(TaskDescriptor& td);
};

extern "C" {
int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();
}
