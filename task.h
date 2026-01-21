#pragma once
#include "slab_allocator.h"
#include <exception>

enum Priority : int { HIGH, MEDIUM, LOW, COUNT };

// Allocated in kernel memory during kernel initialization.
// Every existing task has a TD allocated to it.
struct TaskDescriptor : IntrusiveLinkage<TaskDescriptor> {
  int tid = -1; // task identifier
  int priority = Priority::HIGH;
  TaskDescriptor* parent = nullptr;
  TaskDescriptor* nextReady = nullptr; // next task in the task's ready queue
  TaskDescriptor* nextSend = nullptr;  // next task on the task's send queue
  void* runState = nullptr;            // unknown type
  void* stackPointer = nullptr;        // unknown type
  // TODO: the task's return value, and the task's SPSR (either in TD or stack)
};

extern "C" {
int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();
}
