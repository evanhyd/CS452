#pragma once
#include "marklin/marklin_def.h"
#include "marklin/marklin_measured_data.h"
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "train_track_server_context.h"
#include "train_track_util.h"
#include "user_tasks/send_util.h"
#include "util/debug.h"

namespace k4 {

inline marklin::TrackNodeId pickWanderDestination(TrainTrackServerContext& context, marklin::TrainId trainId,
                                                  marklin::TrackNodeId avoidNodeId) {
  constexpr uint32_t NUM_SENSORS = 80;
  uint32_t seed = context.currentTicks * 1664525u + trainId * 1013904223u;
  if (avoidNodeId < NUM_SENSORS) {
    uint32_t offset = seed % (NUM_SENSORS - 1) + 1;
    return static_cast<marklin::TrackNodeId>((avoidNodeId + offset) % NUM_SENSORS);
  }
  return static_cast<marklin::TrackNodeId>(seed % NUM_SENSORS);
}

inline void scheduleWanderFindingPath(TrainTrackServerContext& context, marklin::TrainId trainId, marklin::Train& train,
                                      marklin::SpeedLevel speedLevel) {
  marklin::TrackNodeId avoidNodeId =
      train.kinematics.estimatedNode ? train.kinematics.estimatedNode->id : marklin::NUM_TRACK_NODES;
  marklin::TrackNodeId dest = pickWanderDestination(context, trainId, avoidNodeId);
  train.navigation.findingPathTask = {dest, 0, speedLevel, false};
  train.navigation.state = marklin::NavigationSystem::State::FindingPath;
  notifyStatusToUI(context.uiTid, "Train %u wandering to sensor %s.", trainId,
                   context.ttState.getTrackNodeById(dest).name);
}

inline void setSpeedCmdHandler(TrainTrackServerContext& context, marklin::TrainId trainId,
                               marklin::SpeedLevel speedLevel) {
  KIT_ASSERT(marklin::isValidTrainId(trainId), "invalid train id");
  KIT_ASSERT(marklin::isValidSpeedLevel(speedLevel), "invalid train speed level");

  // Check busy status.
  initTrain(context, trainId);
  marklin::Train& train = context.ttState.getTrain(trainId);
  if (train.navigation.state != marklin::NavigationSystem::State::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", trainId);
    return;
  }

  // Change speed.
  broadcastTrainSpeedLevel(context, trainId, speedLevel);
  notifyStatusToUI(context.uiTid, "Set train %u to speed %u.", trainId, speedLevel);
}

inline void wanderCmdHandler(TrainTrackServerContext& context, marklin::TrainId trainId,
                             marklin::SpeedLevel speedLevel) {
  KIT_ASSERT(marklin::isValidTrainId(trainId), "invalid train id");
  KIT_ASSERT(marklin::isValidSpeedLevel(speedLevel), "invalid train speed level");

  initTrain(context, trainId);
  marklin::Train& train = context.ttState.getTrain(trainId);
  if (train.navigation.state != marklin::NavigationSystem::State::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", trainId);
    return;
  }

  if (speedLevel == 0) {
    train.navigation.isWandering = false;
    notifyStatusToUI(context.uiTid, "Train %u stopped wandering.", trainId);
    return;
  }

  train.navigation.isWandering = true;
  scheduleWanderFindingPath(context, trainId, train, speedLevel);
  broadcastTrainSpeedLevel(context, trainId, speedLevel);
  notifyStatusToUI(context.uiTid, "Train %u is wandering at speed %u.", trainId, speedLevel);
}

inline void reverseCmdHandler(TrainTrackServerContext& context, marklin::TrainId trainId) {
  KIT_ASSERT(marklin::isValidTrainId(trainId), "invalid train id");

  // Check busy status.
  initTrain(context, trainId);
  marklin::Train& train = context.ttState.getTrain(trainId);
  if (train.navigation.state != marklin::NavigationSystem::State::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", trainId);
    return;
  }

  // Perform reversing.
  train.navigation.state = marklin::NavigationSystem::State::Reversing;
  train.navigation.reversingTask.preReversingSpeedLevel = train.kinematics.offlineSpeedLevel;
  broadcastTrainSpeedLevel(context, trainId, 0);
  notifyStatusToUI(context.uiTid, "Reversing train %u.", trainId);
}

inline void setSwitchCmdHandler(TrainTrackServerContext& context, marklin::SwitchId switchId,
                                marklin::SwitchState state) {
  KIT_ASSERT(marklin::isValidSwitchId(switchId), "invalid switch id");
  broadcastSwitchState(context, switchId, state);
}

inline void setTrackCmdHandler(TrainTrackServerContext& context, marklin::TrackId trackId) {
  KIT_ASSERT(marklin::isValidTrack(trackId), "invalid track id");
  resetContext(context);
  context.ttState.setCurrentTrack(trackId);
  notifyStatusToUI(context.uiTid, "Switched track to %c.", (trackId == 0 ? 'A' : 'B'));
}

inline void sensorEventHandler(TrainTrackServerContext& context, const marklin::SensorTriggeredEvent& sensorEvent) {
  if (sensorEvent.state != marklin::SensorState::Occupied) {
    return;
  }

  // Send sensor to UI.
  SensorHistoryEntry newEntry{};
  newEntry.event = sensorEvent;
  newEntry.ticks = context.currentTicks;
  context.sensorHistory.push(newEntry);
  sendSensorHistoryToUI(context);

  // Obtain the train that triggers the sensor.
  marklin::TrackNode& sensor = context.ttState.getTrackNodeById(sensorEvent.id);

  marklin::TrainId ownerId = [&] -> marklin::TrainId {
    // Priority 1: Explicit Lock Ownership.
    marklin::TrainId trainId = context.pfSystem.getReserver(sensor.id);
    if (trainId != marklin::NO_TRAIN) {
      return trainId;
    }

    // Priority 2: Prediction Matching.
    for (marklin::TrainId id : context.activeTrains) {
      marklin::Train& train = context.ttState.getTrain(id);
      if (train.prediction.sensor && train.prediction.sensor->id == sensor.id) {
        return id;
      }
    }

    // Priority 3: Pair with the train in Lost state.
    for (marklin::TrainId id : context.activeTrains) {
      marklin::Train& train = context.ttState.getTrain(id);
      if (train.kinematics.state == marklin::KinematicsSystem::State::Lost) {
        return id;
      }
    }
    return marklin::NO_TRAIN;
  }();
  if (ownerId == marklin::NO_TRAIN) {
    return;
  }
  marklin::Train& train = context.ttState.getTrain(ownerId);

  marklin::Distance dS = [&] -> marklin::Distance {
    if (train.kinematics.state == marklin::KinematicsSystem::State::Tracked) {
      return marklin::getDistanceBetweenSensor(context.ttState.getCurrentTrackId(), *train.kinematics.lastSensor,
                                               sensor);
    }
    return 0;
  }();

  // Update kinematics values.
  train.kinematics.triggerSensor(sensor, dS, context.currentTicks);

  // Update the sensor prediction.
  marklin::Distance distToNext = 0;
  marklin::TrackNode* nextSensor = marklin::getNextSensor(context.ttState, sensor, distToNext);
  train.prediction.triggerSensor(sensor, nextSensor, distToNext, train.kinematics.estimatedSpeed, context.currentTicks);
}

inline void gotoCmdHandler(TrainTrackServerContext& context, marklin::TrainId id, marklin::SpeedLevel speedLevel,
                           const char* location, marklin::Distance offset) {
  KIT_ASSERT(marklin::isValidTrainId(id), "Invalid train id for goto.");
  KIT_ASSERT(marklin::isValidSpeedLevel(speedLevel), "Invalid speed level for goto.");
  marklin::TrackNode* dest = context.ttState.getTrackNodeByName(location);
  KIT_ASSERT(dest, "Invalid track node name for goto.");

  // Check busy status.
  initTrain(context, id);
  marklin::Train& train = context.ttState.getTrain(id);
  if (train.navigation.state != marklin::NavigationSystem::State::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", id);
    return;
  }

  train.navigation.isWandering = false;
  train.navigation.findingPathTask = {dest->id, offset, speedLevel, false};
  train.navigation.state = marklin::NavigationSystem::State::FindingPath;
  broadcastTrainSpeedLevel(context, id, speedLevel);
  notifyStatusToUI(context.uiTid, "Route train %u to %s.", id, dest->name);
}

inline void timerTickHandler(TrainTrackServerContext& context, uint32_t ticks) {
  context.currentTicks = ticks;
  bool shouldUpdateTrainUI = context.currentTicks - context.lastTrainUIRefreshTicks >= 10;

  for (marklin::TrainId trainId : context.activeTrains) {
    marklin::Train& train = context.ttState.getTrain(trainId);

    // Part A: Kinematics Block.
    train.kinematics.onTick(context.ttState);

    // Part B: Navigation Block.
    switch (train.navigation.state) {
    case marklin::NavigationSystem::State::Manual: {
      break;
    }
    case marklin::NavigationSystem::State::FindingPath: {
      if (train.kinematics.state != marklin::KinematicsSystem::State::Tracked) {
        break;
      }

      // First time checking reverse.
      if (!train.navigation.findingPathTask.needToReverse) {
        if (context.pfSystem.planPath(context.ttState, trainId, train.kinematics.estimatedNode->id,
                                      train.navigation.findingPathTask.dest, train.navigation.findingPathTask.offset)) {
          train.navigation.state = marklin::NavigationSystem::State::Routed;
          notifyStatusToUI(context.uiTid, "Train %u enter routed state.", trainId);
          break;
        } else {
          train.navigation.findingPathTask.needToReverse = true;
          broadcastTrainSpeedLevel(context, trainId, 0);
          break;
        }
      } else {
        // Wait for the reverse to complete.
        if (train.kinematics.estimatedSpeed != 0) {
          break;
        }

        // Update the reversed kinematics state.
        train.kinematics.direction =
            (train.kinematics.direction == marklin::TrainDirection::Forward ? marklin::TrainDirection::Backward
                                                                            : marklin::TrainDirection::Forward);
        sendToDispatcher(context.dispatcherTid,
                         marklin::MMessage::setTrainDirection(trainId, train.kinematics.direction));

        train.kinematics.triggerSensor(*(train.kinematics.lastSensor->reverse), 0, context.currentTicks);
        marklin::Distance distToNext = 0;
        marklin::TrackNode* nextSensor =
            marklin::getNextSensor(context.ttState, *train.kinematics.lastSensor, distToNext);
        train.prediction.triggerSensor(*train.kinematics.lastSensor, nextSensor, distToNext,
                                       train.kinematics.estimatedSpeed, context.currentTicks);
        broadcastTrainSpeedLevel(context, trainId, train.navigation.findingPathTask.maxSpeedLevel);

        if (context.pfSystem.planPath(context.ttState, trainId, train.kinematics.estimatedNode->id,
                                      train.navigation.findingPathTask.dest, train.navigation.findingPathTask.offset)) {
          train.navigation.state = marklin::NavigationSystem::State::Routed;
          notifyStatusToUI(context.uiTid, "Train %u enter routed state.", trainId);
          break;
        } else {
          if (train.navigation.isWandering) {
            scheduleWanderFindingPath(context, trainId, train, train.navigation.findingPathTask.maxSpeedLevel);
            broadcastTrainSpeedLevel(context, trainId, train.navigation.findingPathTask.maxSpeedLevel);
          } else {
            train.navigation.state = marklin::NavigationSystem::State::Manual;
            notifyStatusToUI(context.uiTid, "Train %u can not find a path.", trainId);
          }
          break;
        }
      }
      break;
    }
    case marklin::NavigationSystem::State::Routed: {
      marklin::Distance enterableDistance = 0;
      auto pathFindingState = context.pfSystem.updateState(
          trainId, train, enterableDistance,
          [&](marklin::SwitchId id, marklin::SwitchState st) { broadcastSwitchState(context, id, st); });

      switch (pathFindingState) {
      case marklin::PathFindingSystem::State::IDLING: {
        // Train has no path finding task. Stationary.
        if (train.navigation.isWandering) {
          scheduleWanderFindingPath(context, trainId, train, train.navigation.findingPathTask.maxSpeedLevel);
          broadcastTrainSpeedLevel(context, trainId, train.navigation.findingPathTask.maxSpeedLevel);
        } else {
          train.navigation.state = marklin::NavigationSystem::State::Manual;
        }
        break;
      }
      case marklin::PathFindingSystem::State::MOVING: {
        // Can enter all nodes within the stopping distance.
        // Moving toward the destination.
        broadcastTrainSpeedLevel(context, trainId, train.navigation.findingPathTask.maxSpeedLevel);
        break;
      }
      case marklin::PathFindingSystem::State::YIELDING: {
        // Can not enter all nodes within the stopping distance.
        // Slowing down.
        broadcastTrainSpeedLevel(context, trainId, 0);
        break;
      }
      case marklin::PathFindingSystem::State::ARRIVING: {
        // Can enter all nodes within the stopping distance.
        // Destination is within the stopping distance.
        // Slowing down.
        broadcastTrainSpeedLevel(context, trainId, 0);
        break;
      }
      }
      break;
    }
    case marklin::NavigationSystem::State::Reversing: {
      if (train.kinematics.estimatedSpeed == 0) {
        train.kinematics.direction =
            (train.kinematics.direction == marklin::TrainDirection::Forward ? marklin::TrainDirection::Backward
                                                                            : marklin::TrainDirection::Forward);
        sendToDispatcher(context.dispatcherTid,
                         marklin::MMessage::setTrainDirection(trainId, train.kinematics.direction));

        if (train.kinematics.state == marklin::KinematicsSystem::State::Tracked) {
          train.kinematics.lastSensor = train.kinematics.lastSensor->reverse;
          train.kinematics.lastSensorOffset = -train.kinematics.lastSensorOffset;
          broadcastTrainSpeedLevel(context, trainId, train.navigation.reversingTask.preReversingSpeedLevel);
          train.navigation.state = marklin::NavigationSystem::State::Manual;
        }
      }
      break;
    }
    }

    // Log to UI.
    if (shouldUpdateTrainUI) {
      KIT_ASSERT(!context.trainStates.full(), "train state buffer overflow");
      TrainStatesEntry entry{.trainId = trainId, .train = &train, .lockedNodes = {}, .lockedNodeCount = 0};
      for (auto& node : context.pfSystem.getTrainPath(trainId).nodes) {
        if (context.pfSystem.getReserver(node.srce->id) != trainId) {
          break;
        }
        entry.lockedNodes[entry.lockedNodeCount++] = &context.ttState.getTrackNodeById(node.srce->id);
      }
      context.trainStates.pushBack(entry);
    }
  }

  // Send the train states to the UI.
  if (shouldUpdateTrainUI) {
    context.lastTrainUIRefreshTicks = context.currentTicks;
    sendTrainHistoryToUI(context);
    context.trainStates.clear();
  }
}

} // namespace k4
