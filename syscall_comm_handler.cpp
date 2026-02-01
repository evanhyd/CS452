#include "syscall_comm_handler.h"
#include "kit_algorithm.h" // memcpy
#include "task_manager.h"

namespace syscall_handler {

static constexpr int RET_PLACEHOLDER = -67;

// Sends a message to another task and receives a reply.
// Return Value
// >=0  the size of the message returned by the replying task. The actual reply is less than or equal to the size of the
// reply buffer provided for it. Longer replies are truncated.
// -1	tid is not the task id of an existing task.
// -2   send-receive-reply transaction could not be completed.
int Send(int tid, const char* message, int messageSize, char* replyBuffer, int replyBufferSize) {

  TaskDescriptor* receiver = tidAllocator.getTaskDescriptor(tid);
  if (!receiver) {
    return -1;
  }

  auto currTask = TaskScheduler::getCurrentTask();

  // Set up the message control block.
  currTask->messageControlBlock.message = message;
  currTask->messageControlBlock.messageSize = messageSize;
  currTask->messageControlBlock.receiveBuffer = replyBuffer;
  currTask->messageControlBlock.receiveBufferSize = replyBufferSize;

  if (receiver->runState == RunState::RECEIVE_WAIT) {
    // Copy the data to the receiver.
    int transferSize = kit::min(messageSize, receiver->messageControlBlock.receiveBufferSize);
    memcpy(receiver->messageControlBlock.receiveBuffer, message, size_t(transferSize));
    *(receiver->messageControlBlock.senderTid) = currTask->tid.raw();

    // Move sender from ready to reply wait.
    TaskScheduler::removeTask(*currTask);
    currTask->runState = RunState::REPLY_WAIT;

    // Move receiver from receive wait to ready.
    TaskScheduler::enqueTask(*receiver);
    receiver->runState = RunState::READY;
    receiver->setRetValue(messageSize);

  } else {
    // Move sender from ready to send wait.
    TaskScheduler::removeTask(*currTask);
    receiver->sendWaitQueue.enque(*currTask);
    currTask->runState = RunState::SEND_WAIT;
  }

  return RET_PLACEHOLDER;
}

// Returns with the sent message in its message buffer and tid set to the task id of the task that sent the
// message.
// Return Value
// >= 0 the size of the message sent by the sender(stored in tid).The actual message is less than
// or equal to the size of the message buffer supplied. Longer messages are truncated.
int Receive(int* tid, char* receiveBuffer, int receiveBufferSize) {
  auto currTask = TaskScheduler::getCurrentTask();

  // Set up the message control block.
  currTask->messageControlBlock.receiveBuffer = receiveBuffer;
  currTask->messageControlBlock.receiveBufferSize = receiveBufferSize;
  currTask->messageControlBlock.senderTid = tid;

  TaskDescriptor* sender = currTask->sendWaitQueue.pop();
  if (!sender) {
    // No messages in the queue, block the task.
    TaskScheduler::removeTask(*currTask);
    currTask->runState = RunState::RECEIVE_WAIT;
    return RET_PLACEHOLDER;
  }

  if (sender->runState != RunState::SEND_WAIT) {
    logError("sender not in SEND_WAIT state");
  }

  // Copy sender's message.
  *tid = sender->tid.raw();
  int transferSize = kit::min(receiveBufferSize, sender->messageControlBlock.messageSize);
  memcpy(receiveBuffer, sender->messageControlBlock.message, size_t(transferSize));

  // Move the sender to replyWait.
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

  TaskDescriptor* sender = tidAllocator.getTaskDescriptor(tid);
  // Invalid task.
  if (!sender) {
    return -1;
  }

  // Task not in reply-wait.
  if (sender->runState != RunState::REPLY_WAIT) {
    return -2;
  }

  // Copy the reply message.
  int transferSize = kit::min(replySize, sender->messageControlBlock.receiveBufferSize);
  memcpy(sender->messageControlBlock.receiveBuffer, reply, size_t(transferSize));

  // Move sender to readyQueue.
  TaskScheduler::enqueTask(*sender);
  sender->runState = RunState::READY;
  sender->setRetValue(transferSize);

  return transferSize;
}

} // namespace syscall_handler
