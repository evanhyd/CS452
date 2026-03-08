#pragma once
#include "util/debug.h"
#include <cstddef>

// A double-ended cyclic queue.
template <typename T, size_t capacity> class RingBuffer {
  T data[capacity];
  size_t sz = 0;
  size_t head = 0;

public:
  // Push to the back.
  void pushBack(const T& value) {
    if (sz == capacity) {
      logError("ring buffer is full");
    }
    size_t tail = (head + sz) % capacity;
    data[tail] = value;
    ++sz;
  }

  // Pop from the back.
  T popBack() {
    if (sz == 0) {
      logError("ring buffer is empty");
    }
    size_t tail = (head + sz - 1) % capacity;
    --sz;
    return data[tail];
  }

  // Access the back element.
  T& back() {
    if (sz == 0) {
      logError("ring buffer is empty");
    }
    size_t tail = (head + sz - 1) % capacity;
    return data[tail];
  }

  const T& back() const {
    if (sz == 0) {
      logError("ring buffer is empty");
    }
    size_t tail = (head + sz - 1) % capacity;
    return data[tail];
  }

  // Push to the front.
  void pushFront(const T& value) {
    if (sz == capacity) {
      logError("ring buffer is full");
    }
    head = (head - 1 + capacity) % capacity;
    data[head] = value;
    ++sz;
  }

  // Pop from the front.
  T popFront() {
    if (sz == 0) {
      logError("ring buffer is empty");
    }

    T value = data[head];
    head = (head + 1) % capacity;
    --sz;
    return value;
  }

  // Access the front element.
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
  void clear() {
    head = 0;
    sz = 0;
  }
};
