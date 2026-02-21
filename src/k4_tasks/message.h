#pragma once
#include "marklin/marklin_message.h"
#include <cstdint>

namespace k4 {

constexpr uint32_t NOT_ACKED = 0xFFFFFFFF;

struct CmdHistoryEntry {
  marklin::MMessage msg;
  unsigned sentTicks; // clock ticks when sent
  uint32_t ackAfter;  // ticks elapsed until ack, or NOT_ACKED
};

inline constexpr const char* MAIN_SERVER_NAME = "k4_server";

enum class MessageType : int { Timer, KeyPress, SensorEvent, CanResponse, StartReverse, EndReverse };

struct EmptyData {};

struct TimerUpdate {
  unsigned deciseconds;
};

struct KeyPressData {
  char c;
};

struct SensorEventData {
  char bank;
  uint8_t number;
  bool oldOccupied;
  bool newOccupied;
};

struct CanResponseData {
  marklin::MMessage msg;
};

struct EndReverseData {
  uint8_t trainNo;
};

struct Message {
  MessageType type;
  union {
    EmptyData empty;
    TimerUpdate timerUpdate;
    KeyPressData keyPress;
    SensorEventData sensorEvent;
    CanResponseData canResponse;
    EndReverseData endReverse;
  };
};

} // namespace k4
