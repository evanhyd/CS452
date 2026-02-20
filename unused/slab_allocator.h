#pragma once
#include "util/debug.h"
#include "util/kit_algorithm.h"
#include <cstddef>
#include <cstdint>
#include <new>

// Slab allocator or pool allocator.
// Allocate an array of fixed size memory blocks.
template <typename T, size_t N> class SlabAllocator {
  union IntrusiveLinkage {
    IntrusiveLinkage* next;
    alignas(T) std::byte storage[sizeof(T)];
  };
  IntrusiveLinkage* free;
  IntrusiveLinkage data[N];

public:
  SlabAllocator() : free{&data[0]} {
    for (size_t i = 0; i < N - 1; ++i) {
      data[i].next = &data[i + 1];
    }
    data[N - 1].next = nullptr;
  }

  T* allocate() {
    if (!free) {
      logError("allocator run out of memory");
    }
    IntrusiveLinkage* ptr = free;
    free = free->next;
    return std::launder(reinterpret_cast<T*>(ptr->storage));
  }

  void deallocate(T* ptr) {
    // Ignore nullptr.
    if (!ptr) {
      return;
    }
    IntrusiveLinkage* old = free;
    free = std::launder(reinterpret_cast<IntrusiveLinkage*>(ptr));
    free->next = old;
  }

  bool full() const { return free == nullptr; }
};
