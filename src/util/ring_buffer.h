#pragma once
#include "util/debug.h"
#include <cstddef>

// A FIFO cyclic queue with static size.
template <typename T, size_t capacity> class RingBuffer {
  T data[capacity];
  size_t sz = 0;
  size_t head = 0;

public:
  // Push to the back of the queue.
  void push(const T& value) {
    if (sz == capacity) {
      logError("ring buffer is full");
    }
    size_t tail = (head + sz) % capacity;
    data[tail] = value;
    ++sz;
  }

  // Pop from the front of the queue.
  T pop() {
    if (sz == 0) {
      logError("ring buffer is empty");
    }
    T value = data[head];
    head = (head + 1) % capacity;
    --sz;
    return value;
  }

  T& front() {
    if (sz == 0) {
      logError("ring buffer is empty");
    }
    return data[head];
  }

  const T& front() const {
    if (sz == 0) {
      logError("ring buffer is empty");
    }
    return data[head];
  }

  size_t size() const { return sz; }

  bool empty() const { return sz == 0; }

  bool full() const { return sz == capacity; }
};
