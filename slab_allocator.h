#pragma once
#include "kit_algorithm.h"

template <typename T> struct IntrusiveLinkage {
  T* prev = nullptr;
  T* next = nullptr;
};

// All data must be either static or on the stack.
template <typename T, size_t capacity> class SlabAllocator {
  static_assert(capacity > 0, "capacity must be positive");
  T data[capacity];
  T* freed;
  T* allocated;

public:
  SlabAllocator() : data{}, freed(data), allocated() {
    for (size_t i = 0; i < capacity - 1; ++i) {
      data[i].next = &data[i + 1];
    }
  }

  T* allocate() {
    // Pop from the head of freed.
    T* ptr = freed;
    freed = freed->next;

    // Push to the head of allocated.
    if (allocated) {
      allocated->prev = ptr;
    }
    ptr->prev = nullptr;
    ptr->next = allocated;
    allocated = ptr;
    return ptr;
  }

  void deallocate(T* ptr) {
    // Ignore nullptr.
    if (!ptr) {
      return;
    }

    // Fix parent child links, and allocated link.
    T* parent = ptr->prev;
    T* child = ptr->next;
    if (parent) {
      parent->next = child;
    } else {
      allocated = child;
    }
    if (child) {
      child->prev = parent;
    }

    // Push to the head of freed.
    ptr->next = freed;
    freed = ptr;
  }
};
