#pragma once
#include "marklin/marklin_event.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include <array>
#include <cstdint>

namespace k4 {
inline constexpr size_t SENSOR_HISTORY_SIZE = 16;
inline constexpr size_t CMD_HISTORY_SIZE = 16;
inline constexpr uint32_t NOT_ACKED = 0xFFFFFFFF;

struct CmdHistoryEntry {
  marklin::MMessage msg;
  uint32_t sentTicks; // clock ticks when sent
  uint32_t ackAfter;  // ticks elapsed until ack, or NOT_ACKED
};

struct SensorHistoryEntry {
  marklin::SensorTriggeredEvent event;
  uint32_t ticks;
};

struct TrainStatesEntry {
  marklin::TrainId trainId;
  marklin::Train* train;
  marklin::TrackNode* nodes[marklin::MAX_PATH_NODES];
  unsigned nodeCount;
  unsigned lockCount;
};

// Clock Server Message
struct TimeData {
  uint32_t ticks;
};

inline constexpr const char* MARKLIN_DISPATCHER_SERVER_NAME = "marklin_dispatcher_server";
inline constexpr const char* TRAIN_TRACK_SERVER_NAME = "train_track_server";
inline constexpr const char* UI_VIEW_SERVER_NAME = "ui_view_server";
inline constexpr const char* PACMAN_SERVER_NAME = "pacman_server";

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
  WanderCmd,
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
  struct WanderCmdData {
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
    WanderCmdData wanderCmd;
    ReverseCmdData reverseCmd;
    SetSwitchCmdData setSwitchCmd;
    SetTrackCmdData setTrackCmd;
    GotoCmdData gotoCmd;
    marklin::SensorTriggeredEvent sensorEvent;
    TimeData time;
  };
};

// Pacman Server Message
enum class PacmanMsgType : int {
  TimerTick,
  GameStateUpdate,
};

inline constexpr marklin::TrackNodeId INVALID_TRACK_NODE_ID =
    static_cast<marklin::TrackNodeId>(marklin::NUM_TRACK_NODES);

struct PacmanTrainStateEntry {
  marklin::TrainId trainId;
  marklin::TrackNodeId estimatedNodeId;
  marklin::TrackNodeId lastSensorId;
  marklin::TrainDirection direction;
  marklin::Distance estimatedNodeOffset;
  marklin::Distance lastSensorOffset;
  bool isTracked;
};

struct PacmanMsg {
  PacmanMsgType type;

  struct GameStateUpdateData {
    uint32_t ticks;
    marklin::TrackId trackId;
    PacmanTrainStateEntry entries[marklin::NUM_TRAIN_IN_LAB];
    unsigned count;
  };

  union {
    TimeData time;
    GameStateUpdateData gameStateUpdate;
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
    std::array<char, 128> msg;
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
    TrainStatesEntry entries[marklin::NUM_TRAIN_IN_LAB];
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
