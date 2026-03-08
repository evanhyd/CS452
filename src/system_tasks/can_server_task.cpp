#include "can_server_task.h"

#include "kernel/devices/mcp2515.h"
#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "util/debug.h"
#include "util/ring_buffer.h"

namespace {

enum class CanServerMessageType : int {
  ReceiveRequest,
  TransmitRequest,
  ReadyNotify,
};

struct ReceiveRequest {};
struct TransmitRequest {
  marklin::MMessage message;
};
struct ReadyNotify {};

struct CanServerMessage {
  CanServerMessageType type;
  union {
    ReceiveRequest receiveRequest;
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
  RingBuffer<int, BUFFER_SIZE> receiveWaitingQueue;
  RingBuffer<marklin::MMessage, BUFFER_SIZE> receiveBuffer;
  RingBuffer<marklin::MMessage, BUFFER_SIZE> transmitBuffer;
  bool isTX0Ready = true;

  for (;;) {
    int tid;
    CanServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(CanServerMessage));

    switch (msg.type) {
    case CanServerMessageType::ReceiveRequest: {
      // Already received a message.
      if (!receiveBuffer.empty()) {
        marklin::MMessage message = receiveBuffer.popFront();
        ::Reply(tid, reinterpret_cast<const char*>(&message), sizeof(marklin::MMessage));
        break;
      }

      // No message in the buffer, block and wait.
      receiveWaitingQueue.pushBack(tid);
      break;
    }
    case CanServerMessageType::TransmitRequest: {
      if (isTX0Ready) {
        // TX0 is ready, transmit immediately.
        isTX0Ready = false;
        mcp2515::sendMessage(msg.transmitRequest.message);
      } else {
        // TX0 is not ready, append to the buffer.
        transmitBuffer.pushBack(msg.transmitRequest.message);
      }

      ::Reply(tid, "", 0);
      break;
    }
    case CanServerMessageType::ReadyNotify: {
      if (tid != notifierTid) {
        logError("the notify message is not from the CAN notifier task");
      }

      uint8_t interruptFlag = mcp2515::getInterruptFlags();

      // Receive Buffer 0 Full
      if (interruptFlag & mcp2515::CanInterruptMask::RX0IE) {
        marklin::MMessage message = mcp2515::receiveMessage(mcp2515::RxBuffer::Rx0);
        if (!receiveWaitingQueue.empty()) {
          int waitingTask = receiveWaitingQueue.popFront();
          ::Reply(waitingTask, reinterpret_cast<const char*>(&message), sizeof(marklin::MMessage));
        } else {
          receiveBuffer.pushBack(message);
        }
        mcp2515::clearInterrupt(mcp2515::CanInterruptMask::RX0IE);
      }

      // Receive Buffer 1 Full
      if (interruptFlag & mcp2515::CanInterruptMask::RX1IE) {
        marklin::MMessage message = mcp2515::receiveMessage(mcp2515::RxBuffer::Rx1);
        if (!receiveWaitingQueue.empty()) {
          int waitingTask = receiveWaitingQueue.popFront();
          ::Reply(waitingTask, reinterpret_cast<const char*>(&message), sizeof(marklin::MMessage));
        } else {
          receiveBuffer.pushBack(message);
        }
        mcp2515::clearInterrupt(mcp2515::CanInterruptMask::RX1IE);
      }

      // Transmit Buffer 0 Empty
      if (interruptFlag & mcp2515::CanInterruptMask::TX0IE) {
        if (!transmitBuffer.empty()) {
          marklin::MMessage message = transmitBuffer.popFront();
          mcp2515::sendMessage(message);
        } else {
          isTX0Ready = true;
        }
        mcp2515::clearInterrupt(mcp2515::CanInterruptMask::TX0IE);
      }

      ::Reply(notifierTid, "", 0);
      break;
    }
    default:
      logError("invalid request type");
      break;
    }
  }
}

int ReceiveCAN(int tid, marklin::MMessage& msg) {
  CanServerMessage request{.type = CanServerMessageType::ReceiveRequest, .receiveRequest{}};
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(&msg),
             sizeof(marklin::MMessage)) < 0) {
    return -1;
  }
  return 0;
}

int TransmitCAN(int tid, const marklin::MMessage& msg) {
  CanServerMessage request{.type = CanServerMessageType::TransmitRequest, .transmitRequest{msg}};
  int value;
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(&value),
             sizeof(int)) < 0) {
    return -1;
  }
  return value;
}
