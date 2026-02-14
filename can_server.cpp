#include "can_server.h"
#include "debug.h"
#include "mcp2515.h"
#include "ring_buffer.h"
#include "syscalls.h"

namespace {

enum class CanServerMessageType : int { READ_REQUEST, TRANSMIT_REQUEST, READ_NOTIFY, TRANSMIT_NOTIFY };

struct ReadRequest {};

struct TransmitRequest {
  mcp2515::MMessage msg;
};

struct ReadNotify {
  mcp2515::MMessage msg;
};

struct TransmitNotify {};

struct CanServerMessage {
  CanServerMessageType type;
  union {
    ReadRequest readRequest;
    TransmitRequest transmitRequest;
    ReadNotify readNotify;
    TransmitNotify transmitNotify;
  };
};

void readNotifierTask() {
  int serverTid = ::MyParentTid();
  for (;;) {
    // unsigned char ch;
    // while (!Uart::tryGetc(ch)) {
    //   ::AwaitEvent(::EventId::UART_RX);
    // }
    // IoServerMessage msg{.type = IoServerMessageType::GETC_NOTIFY, .getcNotify = GetcNotify{ch}};
    // char devnull;
    // ::Send(serverTid, reinterpret_cast<const char*>(&msg), sizeof(IoServerMessage), &devnull, 0);
  }
}

struct TransmitReply {
  mcp2515::MMessage msg;
};

void transmitNotifierTask() {
  int serverTid = ::MyParentTid();
  for (;;) {
    CanServerMessage msg{.type = CanServerMessageType::TRANSMIT_NOTIFY, .transmitNotify{}};
    TransmitReply reply;
    ::Send(serverTid, reinterpret_cast<const char*>(&msg), sizeof(CanServerMessage), reinterpret_cast<char*>(&reply),
           sizeof(TransmitReply));
    // while (!Uart::tryPutc(reply.ch)) {
    //   ::AwaitEvent(::EventId::UART_TX);
    // }
  }
}

} // namespace

void can_server::canServerTask() {
  if (::RegisterAs(CAN_SERVER_NAME) < 0) {
    logError("can server failed to register itself to name server");
  }

  int getcNotifierTid = ::Create(0, readNotifierTask);
  int putcNotifierTid = ::Create(0, transmitNotifierTask);

  RingBuffer<int, 1024> readWaitingQueue;
  RingBuffer<ReadNotify, 1024> readBuffer;
  RingBuffer<TransmitReply, 1024> toTransmitBuffer;

  bool transmitReady = false;

  for (;;) {
    int tid;
    CanServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(CanServerMessage));

    switch (msg.type) {
    case CanServerMessageType::READ_REQUEST:
      break;
    case CanServerMessageType::TRANSMIT_REQUEST:
      break;
    case CanServerMessageType::READ_NOTIFY:
      break;
    case CanServerMessageType::TRANSMIT_NOTIFY:
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
