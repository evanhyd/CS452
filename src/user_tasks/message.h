#pragma once
#include "marklin/marklin_event.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include <cstdint>

namespace k4 {
inline constexpr unsigned SENSOR_HISTORY_SIZE = 16;
inline constexpr unsigned CMD_HISTORY_SIZE = 16;
inline constexpr uint32_t NOT_ACKED = 0xFFFFFFFF;

struct CmdHistoryEntry {
  marklin::MMessage msg;
  unsigned sentTicks; // clock ticks when sent
  uint32_t ackAfter;  // ticks elapsed until ack, or NOT_ACKED
};

struct SensorHistoryEntry {
  marklin::SensorTriggeredEvent event;
  unsigned ticks;
  bool hasPrediction;
  uint32_t predictedId;
  unsigned predictedTicks;
  int timeErrorTicks;
  int distErrorMm;
};

// Clock Server Message
struct TimeData {
  unsigned ticks;
};

inline constexpr const char* MARKLIN_DISPATCHER_SERVER_NAME = "marklin_dispatcher_server";
inline constexpr const char* TRAIN_TRACK_SERVER_NAME = "train_track_server";
inline constexpr const char* UI_VIEW_SERVER_NAME = "ui_view_server";

// Marklin Dispatcher Server Message
enum class DispatcherMsgType : int {
  SendMsg,
  ReceiveMsg,
  TimerTick,
};

struct DispatcherMsg {
  DispatcherMsgType type;
  union {
    marklin::MMessage mmsg;
    TimeData time;
  };
};

// Train Track Server Message
enum class TrainTrackMsgType : int {
  SetSpeedCmd,
  ReverseCmd,
  SetSwitchCmd,
  SensorEvent,
  TimerTick,
};

struct TrainTrackMsg {
  TrainTrackMsgType type;

  struct SetSpeedCmdData {
    uint8_t trainId;
    uint8_t speed;
  };
  struct ReverseCmdData {
    uint8_t trainId;
  };
  struct SetSwitchCmdData {
    uint8_t switchId;
    marklin::SwitchState state;
  };
  union {
    SetSpeedCmdData setSpeedCmd;
    ReverseCmdData reverseCmd;
    SetSwitchCmdData setSwitchCmd;
    marklin::SensorTriggeredEvent sensorEvent;
    TimeData time;
  };
};

// UI View Server Message
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

struct UIMsg {
  UIMsgType type;

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
    uint8_t switchId;
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
