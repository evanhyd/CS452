#pragma once

#include "message.h"
#include "util/ctfmt.h"
#include "util/history.h"
#include "util/ring_buffer.h"
#include <array>
#include <cstdint>

namespace marklin {
struct MMessage;
enum class SwitchState : uint8_t;
} // namespace marklin

namespace k4 {

constexpr unsigned MAX_TRAINS = 60;
constexpr unsigned NUM_SWITCHES = 22;
constexpr unsigned SENSOR_HISTORY_SIZE = 16;
constexpr unsigned CMD_HISTORY_SIZE = 16;

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

struct SensorHistoryEntry {
  SensorEventData event;
  unsigned ticks;
};

struct ServerState {
  // train stuff
  std::array<TrainState, MAX_TRAINS + 1> trains;
  // need a way to send train number to reverse worker task, so create, then reply with train number
  RingBuffer<uint8_t, 32> toReverse;

  // switch states
  std::array<marklin::SwitchState, NUM_SWITCHES> switches;

  // sensor event history
  History<SensorHistoryEntry, SENSOR_HISTORY_SIZE> sensorHistory;

  // command history (tracks sent CAN messages + acks)
  History<CmdHistoryEntry, CMD_HISTORY_SIZE> cmdHistory;

  // queue commands when waiting for ack
  RingBuffer<marklin::MMessage, 256> canSendBuffer;

  Status status;
  const int canServerTid;
  unsigned currentTicks;

  // send a CAN message, buffering if last command not yet acked
  void sendCAN(const marklin::MMessage& msg);
  // flushes a buffered CAN command if the last command has been acked (or timed out)
  bool tryFlushCanBuffer();
  // check if the latest sent command has been acked (or timed out)
  bool lastCommandAcked() const;
  // process an incoming CAN response: match against cmd history for ack
  bool processCanResponse(const marklin::MMessage& msg);

  // lazily set initial state for a train
  void initTrain(uint8_t trainNo);

  marklin::SwitchState getSwitchState(unsigned id) const;
  void setSwitchState(unsigned id, marklin::SwitchState state);
};

void mainServerTask();

} // namespace k4
