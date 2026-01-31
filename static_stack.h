#pragma once
#include "debug.h"
#include <cstddef>

// LIFO static size stack container.
template <typename T, size_t capacity> class StaticStack {
  T data[capacity];
  size_t size = 0;

public:
  bool empty() const { return size == 0; }

  bool full() const { return size == capacity; }

  void push(const T& t) {
    if (size == capacity) {
      logError("push to a full stack");
    }
    data[size] = t;
    ++size;
  }

  T pop() {
    if (size == 0) {
      logError("pop from empty stack");
    }
    --size;
    T value = data[size];
    return value;
  }

  T& top() {
    if (size == 0) {
      logError("access top from empty stack");
    }
    return data[size - 1];
  }

  const T& top() const {
    if (size == 0) {
      logError("access top from empty stack");
    }
    return data[size - 1];
  }

  T* begin() { return data; }
  T* end() { return data + size; }
  const T* begin() const { return data; }
  const T* end() const { return data + size; }
};
