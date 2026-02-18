#include "can_server.h"
#include "debug.h"
#include "kit_algorithm.h"
#include "marklin_message.h"
#include "mcp2515.h"
#include "ring_buffer.h"
#include "syscalls.h"

namespace {

enum class CanServerMessageType : int {
  ReadRequest,
  TransmitRequest,
  ReadyNotify,
};

struct ReadRequest {};
struct TransmitRequest {
  marklin::MMessage message;
};
struct ReadyNotify {};

struct CanServerMessage {
  CanServerMessageType type;
  union {
    ReadRequest readRequest;
    TransmitRequest transmitRequest;
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
  RingBuffer<marklin::MMessage, BUFFER_SIZE> receiveBuffer;
  RingBuffer<marklin::MMessage, BUFFER_SIZE> transmitBuffer;
  bool isTX0Ready = false;

  for (;;) {
    int tid;
    CanServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(CanServerMessage));

    switch (msg.type) {
    case CanServerMessageType::ReadRequest: {
      // Already received a message.
      if (!receiveBuffer.empty()) {
        marklin::MMessage message = receiveBuffer.pop();
        ::Reply(tid, reinterpret_cast<const char*>(&message), sizeof(marklin::MMessage));
        break;
      }

      // No message in the buffer, block and wait.
      if (readWaitingQueue.full()) {
        logError("CAN server read waiting queue is full");
      }
      readWaitingQueue.push(tid);
      break;
    }
    case CanServerMessageType::TransmitRequest: {
      if (isTX0Ready) {
        // TX0 is ready, transmit immediately.
        isTX0Ready = false;
        mcp2515::sendMessage(msg.transmitRequest.message);
      } else {
        // TX0 is not ready, append to the buffer.
        if (transmitBuffer.full()) {
          logError("transmit buffer is full");
        }
        transmitBuffer.push(msg.transmitRequest.message);
      }

      ::Reply(tid, "", 0);
      mcp2515::clearInterrupt(mcp2515::CanInterruptMask::TX0IE);
      break;
    }
    case CanServerMessageType::ReadyNotify: {
      uint8_t interruptFlag = mcp2515::getInterruptFlags();

      // Receive Buffer 0 Full
      if (interruptFlag & mcp2515::CanInterruptMask::RX0IE) {
      }

      // Receive Buffer 1 Full
      if (interruptFlag & mcp2515::CanInterruptMask::RX1IE) {
      }

      // Transmit Buffer 0 Empty
      if (interruptFlag & mcp2515::CanInterruptMask::TX0IE) {
      }
      break;
    }
    default:
      break;
    }
  }
}

extern "C" int ReadCAN(int tid, marklin::MMessage* msg) {
  CanServerMessage request{.type = CanServerMessageType::ReadRequest, .readRequest{}};
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(msg),
             sizeof(marklin::MMessage)) < 0) {
    return -1;
  }
  return 0;
}

extern "C" int TransmitCAN(int tid, const marklin::MMessage* msg) {
  CanServerMessage request{.type = CanServerMessageType::TransmitRequest, .transmitRequest{*msg}};
  int value;
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(&value),
             sizeof(int)) < 0) {
    return -1;
  }
  return value;
}
