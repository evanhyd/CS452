#pragma once
#include "slab_allocator.h"
#include "task_queue.h"
#include <bit>
#include <concepts>
#include <cstddef>

namespace syscall_handler {

int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();

} // namespace syscall_handler
