#pragma once
#include <cstddef>
#include <cstdint>

#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "user_tasks/message.h"
#include "util/history.h"
#include "util/ring_buffer.h"

namespace k4 {
struct TrainTrackServerContext {
  int dispatcherTid = 0;
  int uiTid = 0;

  uint32_t currentTicks = 0;
  marklin::TrainTrackState ttState{};
  marklin::PathFindingSystem pfSystem{};
  RingBuffer<marklin::TrainId, marklin::NUM_TRAIN_IN_LAB> activeTrains{}; // we have like 8 trains at most lol.
  History<SensorHistoryEntry, SENSOR_HISTORY_SIZE> sensorHistory{};
  RingBuffer<TrainStatesEntry, marklin::NUM_TRAIN_IN_LAB> trainStates{};
  uint32_t lastTrainUIRefreshTicks = 0;
};
} // namespace k4
