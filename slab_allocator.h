#pragma once
#include "kit_algorithm.h"
#include <cstdint>

// Slab allocator or pool allocator.
// Allocate a set of fixed size memory block.
// Can be used for trolling.
class SlabAllocator {
  struct IntrusiveLinkage {
    IntrusiveLinkage* next;
  };
  IntrusiveLinkage* data;

public:
  SlabAllocator(void* beginAddress, void* endAddress, size_t blockSize) : data((IntrusiveLinkage*)(beginAddress)) {
    uintptr_t bytes = reinterpret_cast<uintptr_t>(endAddress) - reinterpret_cast<uintptr_t>(beginAddress);
    size_t capacity = bytes / blockSize;

    for (size_t i = 0; i < capacity - 1; ++i) {
      IntrusiveLinkage* curr = (IntrusiveLinkage*)((uint8_t*)(data) + blockSize * i);
      IntrusiveLinkage* child = (IntrusiveLinkage*)((uint8_t*)(data) + blockSize * (i + 1));
      curr->next = child;
    }

    IntrusiveLinkage* last = (IntrusiveLinkage*)((uint8_t*)(data) + blockSize * (capacity - 1));
    last->next = nullptr;
  }

  void* allocate() {
    void* ptr = data;
    data = data->next;
    return ptr;
  }

  void deallocate(void* ptr) {
    // Ignore nullptr.
    if (!ptr) {
      return;
    }

    static_cast<IntrusiveLinkage*>(ptr)->next = data;
    data = static_cast<IntrusiveLinkage*>(ptr);
  }
};
