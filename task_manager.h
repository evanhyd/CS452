#pragma once

#include "gic.h"
#include "syscalls.h"
#include "task_queue.h"
#include <bit>
#include <cstddef>
#include <cstdint>

inline constexpr size_t MAX_TASK_COUNT = 128;

class Tid {
  static constexpr int MASK_WIDTH = std::bit_width(MAX_TASK_COUNT - 1);
  static constexpr size_t INDEX_MASK = (1uz << MASK_WIDTH) - 1;

public:
  constexpr Tid() : value{} {}
  constexpr Tid(unsigned generation, size_t index)
      : value{(generation << MASK_WIDTH) | static_cast<unsigned>(index & INDEX_MASK)} {}

  static constexpr Tid fromRaw(int raw) {
    unsigned value = static_cast<unsigned>(raw);
    return Tid{value >> MASK_WIDTH, value & INDEX_MASK};
  }
  static constexpr Tid invalid() { return fromRaw(-1); }

  constexpr int raw() const { return static_cast<int>(value); }
  constexpr unsigned generation() const { return value >> MASK_WIDTH; }
  constexpr size_t index() const { return value & INDEX_MASK; }

  struct TaskDescriptor* descriptor() const;
  struct TaskStack* stack() const;

  constexpr friend bool operator==(Tid, Tid) = default;

private:
  unsigned value;
};

// Define the size of the task stack frame.
// Configure TASK_STACK_SIZE to change the task stack size.
struct TaskStack {
  static constexpr size_t TASK_STACK_SIZE = 1 << 20;
  alignas(16) std::byte data[TASK_STACK_SIZE];
  void* top() { return data + TASK_STACK_SIZE; }
  void* offsetFromTop(size_t bytes) { return data + TASK_STACK_SIZE - bytes; }
};

// Define the saved context of each user task.
struct StackContext {
  uint64_t elr_el1;
  uint64_t spsr_el1;
  uint64_t x30;
  uint64_t esr_el1;
  uint64_t x28, x29;
  uint64_t x26, x27;
  uint64_t x24, x25;
  uint64_t x22, x23;
  uint64_t x20, x21;
  uint64_t x18, x19;
  uint64_t x16, x17;
  uint64_t x14, x15;
  uint64_t x12, x13;
  uint64_t x10, x11;
  uint64_t x8, x9;
  uint64_t x6, x7;
  uint64_t x4, x5;
  uint64_t x2, x3;
  uint64_t x0, x1;
};
static_assert(sizeof(StackContext) % 16 == 0, "sp must aligned to 16");

// Define the message format.
struct MessageControlBlock {
  const char* message;
  int messageSize;
  char* receiveBuffer;
  int receiveBufferSize;
  int* senderTid;
};

enum class RunState : int {
  Ready,
  SendBlocked,
  ReceiveBlocked,
  ReplyBlocked,
  EventBlocked,
};

// The handle to an allocated task. Contains all the meta data.
// Allocated in kernel memory during kernel initialization.
struct TaskDescriptor {
  Tid tid; // task identifier
  int priority;
  Tid parentTid;
  TaskStack* stackMemory;
  TaskDescriptor* next; // next task in the queue
  StackContext* stackPointer;
  RunState runState;
  MessageControlBlock messageControlBlock;
  RoundRobinQueue sendWaitQueue;

  void setRetValue(std::convertible_to<uint64_t> auto retValue) { stackPointer->x0 = static_cast<uint64_t>(retValue); }
};

class TidAllocator {
public:
  constexpr TidAllocator() : top{MAX_TASK_COUNT}, generations{} {
    for (unsigned i = 0; i < MAX_TASK_COUNT; ++i) {
      free[i] = MAX_TASK_COUNT - i - 1;
    }
  }

  bool full() const { return top == 0; }

  // precondition: not full()
  Tid allocate();

  void deallocate(Tid tid);

  bool isAlive(Tid tid) const;

  // Return the TaskDescriptor for the tid, or nullptr if it doesn't exist or has exited.
  TaskDescriptor* getTaskDescriptor(int rawTid) const;

private:
  size_t top;
  size_t free[MAX_TASK_COUNT];
  unsigned generations[MAX_TASK_COUNT];
};

inline constinit TidAllocator tidAllocator{};

// A singleton class that schedules the tasks.
// Internally, it uses a multi-level round robin queue.
struct TaskScheduler {
  TaskScheduler() = delete;

  // Return the task descriptor of the currently running task, or nullptr if there isn't any.
  static TaskDescriptor* getCurrentTask();

  // Return the task descriptor of the next scheduled task, or nullptr if there isn't any.
  static TaskDescriptor* getNextScheduledTask();

  // Enque the task to the ready task.
  static void enqueReadyTask(TaskDescriptor& td);

  // Move the given task to the end of the ready priority queue.
  static void moveReadyTaskToEnd(TaskDescriptor& td);

  // Remove the given task from ready priority queue.
  static void removeReadyTask(TaskDescriptor& td);

  // Enque the task to the event blocked queue partitioned by eventId.
  static void enqueEventBlockedTask(::EventId eventId, TaskDescriptor& td);

  // Notify the event blocked tasks, and move all to the ready queue.
  static void notifyAllEventBlockedTasks(::EventId eventId, int eventValue);

  // Context switch to the task denoted by its task descriptor.
  static void activateTask [[noreturn]] (TaskDescriptor& td);
};

void createIdleTask();
