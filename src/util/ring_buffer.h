#pragma once
#include "util/debug.h"
#include <cstddef>
#include <iterator>
#include <type_traits>

// A double-ended cyclic queue.
template <typename T, size_t capacity> class RingBuffer {
  T data[capacity];
  size_t sz = 0;
  size_t head = 0;

  template <bool IsConst> class Iterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = std::conditional_t<IsConst, const T*, T*>;
    using reference = std::conditional_t<IsConst, const T&, T&>;
    using BufferPtr = std::conditional_t<IsConst, const RingBuffer*, RingBuffer*>;

  private:
    BufferPtr buffer;
    size_t i;

  public:
    Iterator(BufferPtr ptr, size_t index) : buffer(ptr), i(index) {}
    reference operator*() const { return buffer->data[(buffer->head + i) % capacity]; }
    pointer operator->() const { return &operator*(); }
    Iterator& operator++() {
      i++;
      return *this;
    }
    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }
    bool operator==(const Iterator& other) const { return buffer == other.buffer && i == other.i; }
    bool operator!=(const Iterator& other) const { return !(*this == other); }
  };

public:
  using iterator = Iterator<false>;
  using const_iterator = Iterator<true>;

  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, sz); }
  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const { return const_iterator(this, sz); }

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
