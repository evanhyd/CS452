#include "can_server.h"
#include "debug.h"
#include "kit_algorithm.h"
#include "mcp2515.h"
#include "ring_buffer.h"
#include "syscalls.h"

namespace {

enum class CanServerMessageType : int {
  ReadRequest,
  ReadResponse,
  TransmitRequest,
  TransmitResponse,
  ReadyNotify,
};

struct ReadRequest {};
struct ReadResponse {
  mcp2515::MMessage message;
};

struct TransmitRequest {
  mcp2515::MMessage msg;
};
struct TransmitResponse {};

struct ReadyNotify {};

struct CanServerMessage {
  CanServerMessageType type;
  union {
    ReadRequest readRequest;
    ReadResponse readResponse;
    TransmitRequest transmitRequest;
    TransmitResponse transmitResponse;
    ReadyNotify readyNotify;
  };
};

void notifierTask() {
  int serverTid = ::MyParentTid();
  for (;;) {
    ::AwaitEvent(::EventId::CanIO);
    CanServerMessage msg{.type = CanServerMessageType::ReadyNotify, .readyNotify{}};
    char dummy;
    ::Send(serverTid, reinterpret_cast<const char*>(&msg), sizeof(CanServerMessage), &dummy, 0);
  }
}

} // namespace

void can_server::canServerTask() {
  if (::RegisterAs(CAN_SERVER_NAME) < 0) {
    logError("can server failed to register itself to name server");
  }

  int notifierTid = ::Create(0, notifierTask);

  constexpr size_t BUFFER_SIZE = 1024;
  RingBuffer<int, BUFFER_SIZE> readWaitingQueue;
  RingBuffer<mcp2515::MMessage, BUFFER_SIZE> receiveBuffer;
  RingBuffer<mcp2515::MMessage, BUFFER_SIZE> transmitBuffer;

  for (;;) {
    int tid;
    CanServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(CanServerMessage));

    switch (msg.type) {
    case CanServerMessageType::ReadRequest:
      // Already received a full message.
      if (receiveBuffer.size() >= SERIALIZED_CAN_MESSAGE_SIZE) {
        // TODO: deserialize the message
        // TODO: reply to the waiting task
        break;
      }

      // Not enough received data, add to the waiting queue.
      if (readWaitingQueue.full()) {
        logError("can server read waiting queue is full");
      }
      readWaitingQueue.push(tid);
      break;
    case CanServerMessageType::TransmitRequest:
      if (BUFFER_SIZE - transmitBuffer.size() < SERIALIZED_CAN_MESSAGE_SIZE) {
        logError("transmit buffer is full");
      }

      // TODO: serialize the message and add to the transmit buffer
      // TODO: if ready to transmit, transmit one byte, and reply to the notifier.
      // TODO: reply to the sender anyway
      break;
    case CanServerMessageType::ReadyNotify:
      break;
    default:
      break;
    }
  }
}

extern "C" int ReadCAN(int tid, mcp2515::MMessage* msg) {
  CanServerMessage request{.type = CanServerMessageType::ReadRequest, .readRequest{}};
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(msg),
             sizeof(mcp2515::MMessage)) < 0) {
    return -1;
  }
  return 0;
}

extern "C" int TransmitCAN(int tid, const mcp2515::MMessage* msg) {
  CanServerMessage request{.type = CanServerMessageType::TransmitRequest, .transmitRequest{*msg}};
  int value;
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(&value),
             sizeof(int)) < 0) {
    return -1;
  }
  return value;
}
