#pragma once

#include "util/ctfmt.h"

#include "util/ring_buffer.h"
#include <array>
#include <cstdint>

namespace marklin {
struct MMessage;
}

namespace k4 {

constexpr unsigned MAX_TRAINS = 60;

struct TrainState {
  uint8_t speed;
  bool forward;
  enum class State : uint8_t {
    Idle,
    Reversing,
  } state;
  bool touched;
};

struct Status {
  char msg[128];
  bool dirty;

  template <typename... Args> void set(kit::FormatSpec<Args...> fmt, const Args&... args) {
    kit::formatString(msg, fmt, args...);
    dirty = true;
  }
};

struct ServerState {
  std::array<TrainState, MAX_TRAINS + 1> trains;
  RingBuffer<uint8_t, 16> toReverse;
  Status status;
  const int canServerTid;

  void sendCAN(const marklin::MMessage& msg);
  void initTrain(uint8_t trainNo);
};

void mainServerTask();

} // namespace k4
