#include "can_server.h"
#include "debug.h"
#include "mcp2515.h"
#include "ring_buffer.h"
#include "syscalls.h"

namespace {

enum class CanServerMessageType : int { READ_REQUEST, TRANSMIT_REQUEST, NOTIFY };

struct ReadRequest {};

struct TransmitRequest {
  mcp2515::MMessage msg;
};

struct Notify {};

struct CanServerMessage {
  CanServerMessageType type;
  union {
    ReadRequest readRequest;
    TransmitRequest transmitRequest;
    Notify notify;
  };
};

void notifierTask() {
  int serverTid = ::MyParentTid();
  for (;;) {
    ::AwaitEvent(::EventId::CAN_IO);
    CanServerMessage msg{.type = CanServerMessageType::NOTIFY, .notify{}};
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

  RingBuffer<int, 1024> readWaitingQueue;
  RingBuffer<mcp2515::MMessage, 1024> readBuffer;
  RingBuffer<mcp2515::MMessage, 1024> toTransmitBuffer;

  for (;;) {
    int tid;
    CanServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(CanServerMessage));

    switch (msg.type) {
    case CanServerMessageType::READ_REQUEST:
      break;
    case CanServerMessageType::TRANSMIT_REQUEST:
      break;
    case CanServerMessageType::NOTIFY:
      break;
    default:
      break;
    }
  }
}

extern "C" int ReadCAN(int tid, mcp2515::MMessage* msg) {
  CanServerMessage request{.type = CanServerMessageType::READ_REQUEST, .readRequest{}};
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(msg),
             sizeof(mcp2515::MMessage)) < 0) {
    return -1;
  }
  return 0;
}

extern "C" int TransmitCAN(int tid, const mcp2515::MMessage* msg) {
  CanServerMessage request{.type = CanServerMessageType::TRANSMIT_REQUEST, .transmitRequest{*msg}};
  int value;
  if (::Send(tid, reinterpret_cast<const char*>(&request), sizeof(CanServerMessage), reinterpret_cast<char*>(&value),
             sizeof(int)) < 0) {
    return -1;
  }
  return value;
}
