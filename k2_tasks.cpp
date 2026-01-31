#include "k2_tasks.h"
#include "debug.h"
#include "name_server.h"
#include "ring_buffer.h"
#include "syscalls.h"
#include "task_queue.h"
#include <cstdint>

namespace {
enum class RPCMessageType : uint8_t { SIGNUP, PLAY, QUIT };

enum class PlayType : uint8_t { ROCK, PAPER, SCISSOR } type;

struct SignUpMessage {};

struct PlayMessage {
  PlayType type;
};

struct QuitMessage {};

struct RPCMessage {
  RPCMessageType type;
  union {
    SignUpMessage signUpMessage;
    PlayMessage playMessage;
    QuitMessage quitMessage;
  };
};
} // namespace

namespace k2 {

void rpcServerTask() {
  constexpr size_t MAX_PLAYER = 32;
  RingBuffer<int, MAX_PLAYER> waitQueue;
  for (;;) {
    int senderTid;
    RPCMessage msg;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(RPCMessage));

    switch (msg.type) {
    case RPCMessageType::SIGNUP:
      break;
    case RPCMessageType::PLAY:
      break;
    case RPCMessageType::QUIT:
      break;
    default:
      logError("invalid rpc message type");
    }
  }
}

void rpcClientTask() {}

void FirstUserTask() {
  // Creates the name server.
  name_server::createNameServerTask(Priority::MEDIUM);

  // Creates the Rock/Paper/Scissors server.

  // Creates the Rock/Paper/Scissors clients.
}
} // namespace k2
