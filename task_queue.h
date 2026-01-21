#pragma once
#include "task.h"

// Cyclic queue that uses round robin to schedule the tasks.
class RoundRobinQueue {
  TaskDescriptor* head = nullptr;
  TaskDescriptor* tail = nullptr;

public:
  bool empty() const { return !head; }

  void enque(TaskDescriptor& td) {
    if (!head) {
      head = &td;
      tail = &td;
    }
    tail->nextReady = &td;
    tail = &td;
    tail->nextReady = head;
  }

  void pop() {
    if (head == tail) {
      head = nullptr;
      return;
    }
    head = head->nextReady;
    tail->nextReady = head;
  }

  void next() {
    tail = head;
    head = head->nextReady;
  }

  TaskDescriptor& current() { return *head; }
};

// Multi level queues that support different priorities.
template <typename T> class MultiLevelQueue {
  T queues[Priority::COUNT] = {};

public:
  bool empty() const {
    for (const T& queue : queues) {
      if (!queue.empty()) {
        return false;
      }
    }
    return true;
  }

  void enque(TaskDescriptor& td) { queues[td.priority].enque(td); }

  void pop() {
    for (T& queue : queues) {
      if (!queue.empty()) {
        queue.pop();
        break;
      }
    }
  }

  void next() {
    for (T& queue : queues) {
      if (!queue.empty()) {
        queue.next();
        break;
      }
    }
  }

  TaskDescriptor& current() {
    for (T& queue : queues) {
      if (!queue.empty()) {
        return queue.current();
      }
    }
    assert(false);
    for (;;) {
    };
  }
};
