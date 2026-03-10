#pragma once
#include "marklin/marklin_event.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include <cstdint>

namespace k4 {
inline constexpr size_t SENSOR_HISTORY_SIZE = 16;
inline constexpr size_t CMD_HISTORY_SIZE = 16;
inline constexpr size_t TRAIN_HISTORY_SIZE = 6; // can only found 6 trains in the lab
inline constexpr uint32_t NOT_ACKED = 0xFFFFFFFF;

struct CmdHistoryEntry {
  marklin::MMessage msg;
  uint32_t sentTicks; // clock ticks when sent
  uint32_t ackAfter;  // ticks elapsed until ack, or NOT_ACKED
};

struct SensorHistoryEntry {
  marklin::SensorTriggeredEvent event;
  uint32_t ticks;
  bool hasPrediction;
  marklin::TrackNodeId predictedId;
  uint32_t predictedTicks;
  int32_t timeErrorTicks;
  marklin::Distance distErrorMm;
};

struct TrainStatesEntry {
  marklin::TrainId trainId;
  marklin::Speed estimatedSpeed;
  marklin::TrackNode* lastVisitedNode;
  marklin::Distance estimatedOffsetFromLast; // um away from the last visited node.
  marklin::TrackNode* estimatedNode;
  marklin::Distance estimatedOffsetFromEstimatedNode; // um away from the estimated node.
  marklin::Distance estimatedPathDistance;            // um away form the destination.

  marklin::TrackNode* lastTrippedSensor;
  marklin::TrackNode* predictedNextSensor;
  uint32_t predictedNextSensorTicks;
  int32_t lastTimeErrorTicks;
  marklin::Distance lastDistErrorUm;
};

// Clock Server Message
struct TimeData {
  uint32_t ticks;
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
  SetTrackCmd,
  GotoCmd,
  SensorEvent,
  TimerTick,
};

struct TrainTrackMsg {
  TrainTrackMsgType type;

  struct SetSpeedCmdData {
    marklin::TrainId trainId;
    marklin::SpeedLevel speedLevel;
  };
  struct ReverseCmdData {
    marklin::TrainId trainId;
  };
  struct SetSwitchCmdData {
    marklin::SwitchId switchId;
    marklin::SwitchState state;
  };
  struct SetTrackCmdData {
    marklin::TrackId trackId;
  };
  struct GotoCmdData {
    marklin::TrainId trainId;
    marklin::SpeedLevel speedLevel;
    marklin::Distance offsetMm;
    char location[8];
  };
  union {
    SetSpeedCmdData setSpeedCmd;
    ReverseCmdData reverseCmd;
    SetSwitchCmdData setSwitchCmd;
    SetTrackCmdData setTrackCmd;
    GotoCmdData gotoCmd;
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
  TrainStates,
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
    marklin::SwitchId switchId;
    marklin::SwitchState state;
  };

  struct SensorHistoryData {
    SensorHistoryEntry entries[SENSOR_HISTORY_SIZE];
    unsigned count;
  };

  struct CmdHistoryData {
    CmdHistoryEntry entries[CMD_HISTORY_SIZE];
    unsigned count;
  };

  struct TrainStatesData {
    TrainStatesEntry entries[TRAIN_HISTORY_SIZE];
    unsigned count;
  };

  union {
    Empty empty;
    PromptInsertData promptInsert;
    PromptDeleteData promptDelete;
    StatusData status;
    TimeData time;
    SwitchUpdateData switchUpdate;
    SensorHistoryData sensorHistory;
    CmdHistoryData cmdHistory;
    TrainStatesData trainStates;
  };
};

} // namespace k4
