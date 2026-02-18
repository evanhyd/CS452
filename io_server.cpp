#include "io_server.h"
#include "debug.h"
#include "ring_buffer.h"
#include "syscalls.h"
#include "uart.h"

namespace {

enum class IoServerMessageType : int { GetcRequest, PutcRequest, GetcNotify, PutcNotify };

struct GetcRequest {};

struct PutcRequest {
  unsigned char ch;
};

struct GetcNotify {
  unsigned char ch;
};

struct PutcNotify {};

struct IoServerMessage {
  IoServerMessageType type;
  union {
    GetcRequest getcRequest;
    PutcRequest putcRequest;
    GetcNotify getcNotify;
    PutcNotify putcNotify;
  };
};

void getcNotifierTask() {
  int serverTid = ::MyParentTid();
  for (;;) {
    unsigned char ch;
    while (!Uart::tryGetc(ch)) {
      ::AwaitEvent(::EventId::UartRx);
    }
    IoServerMessage msg{.type = IoServerMessageType::GetcNotify, .getcNotify = GetcNotify{ch}};
    char devnull;
    ::Send(serverTid, reinterpret_cast<const char*>(&msg), sizeof(IoServerMessage), &devnull, 0);
  }
}

struct PutcReply {
  unsigned char ch;
};

void putcNotifierTask() {
  int serverTid = ::MyParentTid();
  for (;;) {
    IoServerMessage msg{.type = IoServerMessageType::PutcNotify, .putcNotify = PutcNotify{}};
    PutcReply reply;
    ::Send(serverTid, reinterpret_cast<const char*>(&msg), sizeof(IoServerMessage), reinterpret_cast<char*>(&reply),
           sizeof(PutcReply));
    while (!Uart::tryPutc(reply.ch)) {
      ::AwaitEvent(::EventId::UartTx);
    }
  }
}

} // namespace

void io_server::ioServerTask() {
  if (::RegisterAs(IO_SERVER_NAME) < 0) {
    logError("IO server failed to register itself to name server");
  }

  int getcNotifierTid = ::Create(0, getcNotifierTask);
  int putcNotifierTid = ::Create(0, putcNotifierTask);

  RingBuffer<int, 1024> getcWaitingQueue;
  RingBuffer<GetcNotify, 1024> getcBuffer;
  RingBuffer<PutcReply, 1024> toPutcBuffer;

  bool putcReady = false;

  for (;;) {
    int tid;
    IoServerMessage msg;
    ::Receive(&tid, reinterpret_cast<char*>(&msg), sizeof(IoServerMessage));

    switch (msg.type) {
    case IoServerMessageType::GetcRequest:
      // Read from the getc data buffer is not empty.
      // Otherwise wait in the getc tid queue.
      if (!getcBuffer.empty()) {
        int ch = static_cast<int>(getcBuffer.pop().ch);
        ::Reply(tid, reinterpret_cast<const char*>(&ch), sizeof(ch));
      } else {
        if (getcWaitingQueue.full()) {
          logError("io server getc waiting queue is full");
        }
        getcWaitingQueue.push(tid);
      }
      break;
    case IoServerMessageType::PutcRequest:
      // If putcNotifier is ready (waiting for reply), then reply.
      // Otherwise put data in the putc data queue.
      if (putcReady) {
        putcReady = false;
        ::Reply(putcNotifierTid, reinterpret_cast<const char*>(&msg.putcRequest.ch), sizeof(msg.putcRequest.ch));
      } else {
        if (toPutcBuffer.full()) {
          logError("io server putc buffer is full");
        }
        toPutcBuffer.push(PutcReply{msg.putcRequest.ch});
      }
      {
        int success = 0;
        ::Reply(tid, reinterpret_cast<const char*>(&success), sizeof(int));
      }
      break;
    case IoServerMessageType::GetcNotify:
      if (tid != getcNotifierTid) {
        logError("received getc notify from unexpected tid");
      }
      // If has task waiting in the getc tid queue, then reply immediately.
      // Otherwise store the data in the getc data buffer.
      if (!getcWaitingQueue.empty()) {
        int waitingTid = getcWaitingQueue.pop();
        ::Reply(waitingTid, reinterpret_cast<const char*>(&msg.getcNotify.ch), sizeof(msg.getcNotify.ch));
      } else {
        if (getcBuffer.full()) {
          logError("io server getc buffer is full");
        }
        getcBuffer.push(msg.getcNotify);
      }
      ::Reply(getcNotifierTid, "", 0); // reply to getcNotifier
      break;
    case IoServerMessageType::PutcNotify:
      if (tid != putcNotifierTid) {
        logError("received putc notify from unexpected tid");
      }
      // If has data waiting in the putc data queue, then reply to putcNotifier and send to UART.
      // Otherwise mark putcNotifier as ready.
      if (!toPutcBuffer.empty()) {
        PutcReply reply = toPutcBuffer.pop();
        ::Reply(putcNotifierTid, reinterpret_cast<const char*>(&reply.ch), sizeof(reply.ch));
      } else {
        putcReady = true;
      }
      break;
    default:
      break;
    }
  }
}

// Returns the next un-returned character from the terminal. The first argument is the task id of the appropriate I/O
// server.
// Return Value
// >=0	new character from the terminal.
// -1	tid is not a valid terminal server task.
extern "C" int Getc(int tid) {
  IoServerMessage msg{.type = IoServerMessageType::GetcRequest, .getcRequest = GetcRequest{}};
  int value;
  if (::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(IoServerMessage), reinterpret_cast<char*>(&value),
             sizeof(int)) < 0) {
    return -1;
  }
  return value;
}

// Queues the given character for transmission by the terminal. On return the only guarantee is that the character has
// been queued. Whether it has been transmitted or received is not guaranteed.
// Return Value
// 0	success.
// -1	tid is not a valid terminal server task.
extern "C" int Putc(int tid, unsigned char ch) {
  IoServerMessage msg{.type = IoServerMessageType::PutcRequest, .putcRequest = PutcRequest{ch}};
  int value;
  if (::Send(tid, reinterpret_cast<const char*>(&msg), sizeof(IoServerMessage), reinterpret_cast<char*>(&value),
             sizeof(int)) < 0) {
    return -1;
  }
  return value;
}
