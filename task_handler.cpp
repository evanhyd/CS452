#include "task_handler.h"
#include "debug.h"
#include "kit_algorithm.h"
#include "slab_allocator.h"
#include "task.h"
#include <cstdint>
#include <limits>
#include <new>

namespace {

SlabAllocator<TaskDescriptor, 128> taskDescriptorsAllocator{};
SlabAllocator<TaskStack, 128> taskStackAllocator{};

MultiLevelQueue readyQueue{};
MultiLevelQueue receiveWaitQueue{};
MultiLevelQueue replyWaitQueue{};
TaskDescriptor* currentTask_ = nullptr;
int globalTidCounter = 0;

} // namespace

extern "C" [[noreturn]] void switchTask(void* sp);

// Return the task descriptor of the currently running task, or nullptr if there isn't any.
TaskDescriptor* TaskScheduler::getCurrentTask() { return currentTask_; }

// Return the task descriptor of the next scheduled task, or nullptr if there isn't any.
TaskDescriptor* TaskScheduler::getNextScheduledTask() { return readyQueue.current(); }

// Move the given task to the end of its priority queue.
void TaskScheduler::moveTaskToEnd(TaskDescriptor& td) { readyQueue.moveToEnd(td); }

// Remove the given task from its priority queue.
void TaskScheduler::removeTask(TaskDescriptor& td) { readyQueue.remove(td); }

// Context switch to the task denoted by its task descriptor.
void TaskScheduler::activateTask(TaskDescriptor& td) {
  currentTask_ = &td;
  switchTask(td.stackPointer);
}

// Return the TaskDescriptor, or nullptr of doesn't exist.
TaskDescriptor* TaskScheduler::getTaskDescriptor(int tid) {
  // TODO: implement it
  return nullptr;
}

namespace syscall_handler {

// Return the positive integer task id of the newly created task.
// -1 invalid priority.
// -2 kernel is out of task descriptors.
int Create(int priority, void (*function)()) {

  // Check priority.
  if (priority >= Priority::COUNT) {
    return -1;
  }

  // Check if out of task descriptors.
  if (taskDescriptorsAllocator.full()) {
    return -2;
  }

  // Check tid overflow
  if (globalTidCounter == std::numeric_limits<decltype(globalTidCounter)>::max()) {
    return -2;
  }
  ++globalTidCounter;

  // Allocate a new task stack.
  // Initialize all 32 registers: x0-x30, Pstate, ELR.
  // Set up entry to the task wrapper.
  TaskStack* ts = taskStackAllocator.allocate();
  StackContext* context = std::launder(reinterpret_cast<StackContext*>(ts->offsetFromTop(sizeof(StackContext))));
  memset(context, 0, sizeof(StackContext));
  context->elr_el1 = reinterpret_cast<uint64_t>(function);
  context->x30 = reinterpret_cast<uint64_t>(&::Exit);

  // Allocate a new task descriptor.
  TaskDescriptor* td = taskDescriptorsAllocator.allocate();
  TaskDescriptor* currTask = TaskScheduler::getCurrentTask();
  *td = TaskDescriptor{.tid = globalTidCounter,
                       .priority = priority,
                       .parentTid = currTask ? currTask->tid : -1,
                       .stackMemory = ts,
                       .next = nullptr,
                       .stackPointer = context,
                       .runState = RunState::READY,
                       .messageControlBlock = {},
                       .sendWaitQueue = {}};
  readyQueue.enque(*td);
  return globalTidCounter;
}

// Returns the task id of the calling task.
int MyTid() { return TaskScheduler::getCurrentTask()->tid; }

// Returns the task id of the task that created the calling task.
// If the task has no parent (i.e. the initial task created by the kernel), return -1.
// If the parent is dead, it still returns the original parent tid.
int MyParentTid() { return TaskScheduler::getCurrentTask()->parentTid; }

// Causes a task to pause executing.
// The task is moved to the end of its priority queue, and will resume executing when next scheduled.
void Yield() { TaskScheduler::moveTaskToEnd(*TaskScheduler::getCurrentTask()); }

// Causes a task to cease execution permanently. It is removed from all priority queues, send queues, receive queues and
// event queues. Resources owned by the task, primarily its memory and task descriptor, may be reclaimed.
void Exit() {
  auto currTask = TaskScheduler::getCurrentTask();
  TaskScheduler::removeTask(*currTask);
  taskStackAllocator.deallocate(currTask->stackMemory);
  taskDescriptorsAllocator.deallocate(currTask);
}

// Sends a message to another task and receives a reply.
// Return Value
// >=0  the size of the message returned by the replying task. The actual reply is less than or equal to the size of the
// reply buffer provided for it. Longer replies are truncated.
// -1	tid is not the task id of an existing task.
// -2   send-receive-reply transaction could not be completed.
int Send(int tid, const char* message, int messageSize, char* replyBuffer, int replyBufferSize) {

  TaskDescriptor* receiver = TaskScheduler::getTaskDescriptor(tid);
  if (!receiver) {
    return -1;
  }

  // Set up the message control block.
  currentTask_->messageControlBlock.message = message;
  currentTask_->messageControlBlock.messageSize = messageSize;
  currentTask_->messageControlBlock.receiveBuffer = replyBuffer;
  currentTask_->messageControlBlock.receiveBufferSize = replyBufferSize;

  if (receiver->runState == RunState::RECEIVE_WAIT) {
    // Copy the data to the receiver.
    memcpy(receiver->messageControlBlock.receiveBuffer, message, size_t(messageSize));

    // Move sender from ready to reply wait.
    readyQueue.remove(*currentTask_);
    replyWaitQueue.enque(*currentTask_);
    currentTask_->runState = RunState::REPLY_WAIT;

    // Move receiver from receive wait to ready.
    receiveWaitQueue.remove(*receiver);
    readyQueue.enque(*receiver);
    receiver->runState = RunState::READY;

  } else {
    // Move sender from ready to send wait.
    readyQueue.remove(*currentTask_);
    receiver->sendWaitQueue.enque(*currentTask_);
    currentTask_->runState = RunState::SEND_WAIT;
  }

  // TODO: BLOCK
  return 0;
}

// Returns with the sent message in its message buffer and tid set to the task id of the task that sent the
// message.
// Return Value
// >= 0 the size of the message sent by the sender(stored in tid).The actual message is less than
// or equal to the size of the message buffer supplied. Longer messages are truncated.
int Receive(int* tid, char* receiveBuffer, int receiveBufferSize) {
  // Set up the message control block.
  currentTask_->messageControlBlock.receiveBuffer = receiveBuffer;
  currentTask_->messageControlBlock.receiveBufferSize = receiveBufferSize;

  if (currentTask_->sendWaitQueue.empty()) {
    // No messages in the queue, block the task.
    readyQueue.remove(*currentTask_);
    receiveWaitQueue.enque(*currentTask_);
    currentTask_->runState = RunState::RECEIVE_WAIT;
    // TODO: BLOCK
  }

  // Copy sender's message.
  TaskDescriptor* sender = currentTask_->sendWaitQueue.current();
  *tid = sender->tid;
  int transferSize =
      (sender->messageControlBlock.messageSize < receiveBufferSize ? sender->messageControlBlock.messageSize
                                                                   : receiveBufferSize);
  memcpy(receiveBuffer, sender->messageControlBlock.message, size_t(transferSize));

  // Move the sender to replyWait.
  currentTask_->sendWaitQueue.remove(*sender);
  replyWaitQueue.enque(*sender);
  sender->runState = RunState::REPLY_WAIT;
  return transferSize;
}

// Sends a reply to a task that previously sent a message.
// Return Value
// >= 0 the size of the reply message transmitted to the original sender task. If this is less than the size of the
// reply message, the message has been truncated.
// - 1 tid is not the task id of an existing task.
// - 2 tid is not the task id of a reply-blocked task.
int Reply(int tid, const char* reply, int replySize) {

  // Invalid task.
  TaskDescriptor* sender = TaskScheduler::getTaskDescriptor(tid);
  if (!sender) {
    return -1;
  }

  // Task not in reply-wait.
  if (sender->runState != RunState::REPLY_WAIT) {
    return -2;
  }

  // Copy the reply message.
  int transferSize =
      (sender->messageControlBlock.receiveBufferSize < replySize ? sender->messageControlBlock.receiveBufferSize
                                                                 : replySize);
  memcpy(sender->messageControlBlock.receiveBuffer, reply, size_t(transferSize));

  // Move sender to readyQueue.
  currentTask_->sendWaitQueue.remove(*sender);
  readyQueue.enque(*sender);
  sender->runState = RunState::READY;

  return transferSize;
}

} // namespace syscall_handler
