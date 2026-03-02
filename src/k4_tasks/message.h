#pragma once
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include <cstdint>

namespace k4 {

inline constexpr unsigned MAX_TRAINS = 60;
inline constexpr unsigned NUM_SWITCHES = 22;
inline constexpr unsigned SENSOR_HISTORY_SIZE = 16;
inline constexpr unsigned CMD_HISTORY_SIZE = 16;

inline constexpr uint32_t NOT_ACKED = 0xFFFFFFFF;

struct CmdHistoryEntry {
  marklin::MMessage msg;
  unsigned sentTicks; // clock ticks when sent
  uint32_t ackAfter;  // ticks elapsed until ack, or NOT_ACKED
};

struct SensorEventData {
  char bank;
  uint8_t number;
  bool oldOccupied;
  bool newOccupied;
};

struct SensorHistoryEntry {
  SensorEventData event;
  unsigned ticks;
};

struct TimeData {
  unsigned ticks;
};

inline constexpr const char* DISPATCHER_SERVER_NAME = "k4_dispatch";
inline constexpr const char* TRAIN_SERVER_NAME = "k4_train";
inline constexpr const char* TRACK_SERVER_NAME = "k4_track";
inline constexpr const char* UI_SERVER_NAME = "k4_ui";

enum class DispatcherMsgType : int {
  QueueCommand,
  CanResponse,
  TimerTick,
};

struct DispatcherMsg {
  DispatcherMsgType type;
  union {
    marklin::MMessage mmsg;
    TimeData time;
  };
};

enum class TrainMsgType : int {
  SetSpeed,
  Reverse,
  TimerTick,
};

struct TrainMsg {
  TrainMsgType type;
  struct SetSpeedData {
    uint8_t trainNo;
    uint8_t speed;
  };
  struct ReverseData {
    uint8_t trainNo;
  };
  union {
    SetSpeedData setSpeed;
    ReverseData reverse;
    TimeData time;
  };
};

enum class TrackMsgType : int {
  SetSwitch,
  SensorEvent,
  TimerTick,
};

struct TrackMsg {
  TrackMsgType type;
  struct SetSwitchData {
    uint8_t switchNo;
    bool straight;
  };
  union {
    SetSwitchData setSwitch;
    SensorEventData sensorEvent;
    TimeData time;
  };
};

enum class UIMsgType : int {
  PromptInsert,
  PromptDelete,
  PromptClear,
  LogStatus,
  DrawSystemTime,
  DrawIdleTime,
  UpdateSwitch,
  RedrawSensors,
  RedrawCmdHistory,
};

struct Empty {};

struct PromptInsertData {
  unsigned index;
  char ch;
};

struct PromptDeleteData {
  unsigned index;
};

struct StatusData {
  char msg[128];
};

struct SwitchUpdateData {
  uint8_t switchNo;
  marklin::SwitchState state;
};

struct SensorsData {
  SensorHistoryEntry entries[SENSOR_HISTORY_SIZE];
  unsigned count;
};

struct CmdHistoryData {
  CmdHistoryEntry entries[CMD_HISTORY_SIZE];
  unsigned count;
};

struct UIMsg {
  UIMsgType type;
  union {
    Empty empty;
    PromptInsertData promptInsert;
    PromptDeleteData promptDelete;
    StatusData status;
    TimeData time;
    SwitchUpdateData switchUpdate;
    SensorsData sensors;
    CmdHistoryData cmdHistory;
  };
};

} // namespace k4
