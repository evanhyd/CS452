#pragma once
#include "debug.h"
#include "task_handler.h"

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

  void remove(const TaskDescriptor& td) {
    if (head == &td && tail == &td) {
      head = nullptr;
      tail = nullptr;
      return;
    }

    TaskDescriptor* prev = tail;
    TaskDescriptor* curr = head;
    do {
      if (curr == &td) {
        prev->nextReady = curr->nextReady;
        if (head == &td) {
          head = curr->nextReady;
        }
        if (tail == &td) {
          tail = prev;
        }
        return;
      }
      prev = curr;
      curr = curr->nextReady;
    } while (curr != head);

    logError("remove() on a non-existing task");
  }
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
        return;
      }
    }
    logError("pop() on an empty queue");
  }

  void next() {
    for (T& queue : queues) {
      if (!queue.empty()) {
        queue.next();
        return;
      }
    }
    logError("next() on an empty queue");
  }

  TaskDescriptor* current() {
    for (T& queue : queues) {
      if (!queue.empty()) {
        return &queue.current();
      }
    }
    return nullptr;
  }

  void remove(const TaskDescriptor& td) { queues[td.priority].remove(td); }
};
