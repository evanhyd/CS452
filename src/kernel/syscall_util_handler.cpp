#include "syscall_util_handler.h"
#include "task_manager.h"

namespace syscall_handler {
uint64_t GetIdleTime() { return TaskScheduler::getIdleTime(); }
} // namespace syscall_handler
