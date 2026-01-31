#pragma once
#include "debug.h"
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
    T value = data[head];
    head = (head + 1) % capacity;
    --sz;
    return value;
  }

  size_t size() const { return sz; }
};
