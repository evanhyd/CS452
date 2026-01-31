#include "task_queue.h"
#include "task_manager.h"
#include <source_location>

bool RoundRobinQueue::empty() const { return !head; }

void RoundRobinQueue::enque(TaskDescriptor& td) {
  if (!head) {
    head = &td;
    tail = &td;
  }
  tail->next = &td;
  tail = &td;
  tail->next = head;
}

TaskDescriptor* RoundRobinQueue::pop() {
  if (!head) {
    return nullptr;
  }
  TaskDescriptor* ret = head;
  if (head == tail) {
    head = nullptr;
    tail = nullptr;
  } else {
    head = head->next;
    tail->next = head;
  }
  return ret;
}

void RoundRobinQueue::next() {
  tail = head;
  head = head->next;
}

TaskDescriptor& RoundRobinQueue::current() { return *head; }

void RoundRobinQueue::moveToEnd(TaskDescriptor& td) {
  remove(td);
  enque(td);
}

void RoundRobinQueue::remove(TaskDescriptor& td, std::source_location loc) {
  if (head == &td && tail == &td) {
    head = nullptr;
    tail = nullptr;
    return;
  }

  TaskDescriptor* prev = tail;
  TaskDescriptor* curr = head;
  do {
    if (curr == &td) {
      prev->next = curr->next;
      if (head == &td) {
        head = curr->next;
      }
      if (tail == &td) {
        tail = prev;
      }
      return;
    }
    prev = curr;
    curr = curr->next;
  } while (curr != head);

  logError("remove() on a non-existing task", loc);
}

bool MultiLevelQueue::empty() const {
  for (const RoundRobinQueue& queue : queues) {
    if (!queue.empty()) {
      return false;
    }
  }
  return true;
}

void MultiLevelQueue::enque(TaskDescriptor& td) { queues[td.priority].enque(td); }

void MultiLevelQueue::next() {
  for (auto& queue : queues) {
    if (!queue.empty()) {
      queue.next();
      return;
    }
  }
  logError("next() on an empty queue");
}

TaskDescriptor* MultiLevelQueue::current() {
  for (auto& queue : queues) {
    if (!queue.empty()) {
      return &queue.current();
    }
  }
  return nullptr;
}

void MultiLevelQueue::moveToEnd(TaskDescriptor& td) { queues[td.priority].moveToEnd(td); }

void MultiLevelQueue::remove(TaskDescriptor& td) { queues[td.priority].remove(td); }
