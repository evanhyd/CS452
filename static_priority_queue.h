#pragma once

#include <cstddef>
#include <functional>
#include <utility>

// A priority queue with fixed size allocated on stack.
template <typename T, size_t capacity, typename Compare = std::less<T>> class StaticPriorityQueue {
  T data[capacity];
  size_t m_size = 0;
  [[no_unique_address]]
  Compare comp;

  static size_t parent(size_t i) { return (i - 1) / 2; }
  static size_t leftChild(size_t i) { return 2 * i + 1; }
  static size_t rightChild(size_t i) { return 2 * i + 2; }

  void siftUp(size_t i) {
    while (i > 0) {
      size_t p = parent(i);
      if (!comp(data[i], data[p])) {
        break;
      }
      std::swap(data[i], data[p]);
      i = p;
    }
  }

  void siftDown(size_t i) {
    while (true) {
      size_t left = leftChild(i);
      size_t right = rightChild(i);
      size_t x = i;
      if (left < m_size && comp(data[left], data[x])) {
        x = left;
      }
      if (right < m_size && comp(data[right], data[x])) {
        x = right;
      }
      if (x == i) {
        break;
      }
      std::swap(data[i], data[x]);
      i = x;
    }
  }

public:
  explicit StaticPriorityQueue(const Compare& comparator = Compare{}) : comp{comparator} {}

  bool empty() const { return m_size == 0; }
  bool full() const { return m_size == capacity; }
  size_t size() const { return m_size; }
  static constexpr size_t max_size() { return capacity; }
  void clear() { m_size = 0; }

  // precondition: !empty()
  const T& top() const { return data[0]; }

  // precondition: !full()
  void push(const T& value) {
    data[m_size++] = value;
    siftUp(m_size - 1);
  }

  // precondition: !empty()
  void pop() {
    if (--m_size > 0) {
      data[0] = data[m_size];
      siftDown(0);
    }
  }
};
