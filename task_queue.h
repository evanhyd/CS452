#pragma once
#include "debug.h"
#include "syscalls.h"
#include <source_location>

struct TaskDescriptor;

// Cyclic queue that uses round robin to schedule the tasks.
class RoundRobinQueue {
  TaskDescriptor* head = nullptr;
  TaskDescriptor* tail = nullptr;

public:
  bool empty() const;
  void enque(TaskDescriptor& td);
  // return nullptr if empty
  TaskDescriptor* pop();
  void next();
  TaskDescriptor& current();
  void moveToEnd(TaskDescriptor& td);
  void remove(TaskDescriptor& td, std::source_location = std::source_location::current());
};

// Multi level round robin queues that support different priorities.
class MultiLevelQueue {
  RoundRobinQueue queues[MAX_PRIORITY_LEVEL + 1] = {};

public:
  bool empty() const;
  void enque(TaskDescriptor& td);
  void next();
  TaskDescriptor* current();
  void moveToEnd(TaskDescriptor& td);
  void remove(TaskDescriptor& td);
};
