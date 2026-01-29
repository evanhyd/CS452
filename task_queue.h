#pragma once
#include "debug.h"
#include "task.h"

struct TaskDescriptor;

enum Priority : int { HIGH, MEDIUM, LOW, COUNT };

// Cyclic queue that uses round robin to schedule the tasks.
class RoundRobinQueue {
  TaskDescriptor* head = nullptr;
  TaskDescriptor* tail = nullptr;

public:
  bool empty() const;
  void enque(TaskDescriptor& td);
  void pop();
  void next();
  TaskDescriptor& current();
  void moveToEnd(TaskDescriptor& td);
  void remove(TaskDescriptor& td);
};

// Multi level round robin queues that support different priorities.
class MultiLevelQueue {
  RoundRobinQueue queues[Priority::COUNT] = {};

public:
  bool empty() const;
  void enque(TaskDescriptor& td);
  void pop();
  void next();
  TaskDescriptor* current();
  void moveToEnd(TaskDescriptor& td);
  void remove(TaskDescriptor& td);
};
