#pragma once
#include "marklin/marklin_def.h"
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "train_track_server_context.h"
#include "train_track_util.h"
#include "user_tasks/send_util.h"
#include "util/debug.h"
#include "util/kit_algorithm.h"

namespace k4 {

inline constexpr uint32_t FINDING_PATH_RETRY_INITIAL_TICKS = 10;
inline constexpr uint32_t FINDING_PATH_RETRY_MAX_TICKS = 320;

inline marklin::TrackNodeId pickWanderDestination(TrainTrackServerContext& ctx, marklin::TrackNodeId avoidId) {
  avoidId /= 2;
  uint32_t id = ctx.currentTicks % 40;
  for (;;) {
    // Don't duplicate.
    if (id == avoidId) {
      id = (id + 19) % 40;
      continue;
    }

    // Bad sensors.
    if (id == 14 || id == 16 || id == 24 || id == 32) {
      id += 2;
      continue;
    }

    break;
  }

  return marklin::TrackNodeId(id * 2);
}

inline void makeTrainGotoRandomSensor(TrainTrackServerContext& context, marklin::Train& train,
                                      marklin::SpeedLevel speedLevel) {
  marklin::TrackNodeId dest = pickWanderDestination(context, train.navigation.findingPathTask.dest);
  const uint32_t retryAtTicks = train.navigation.findingPathTask.retryAtTicks;
  const uint32_t retryBackoffTicks = train.navigation.findingPathTask.retryBackoffTicks;
  train.navigation.findingPathTask = {dest, 0, speedLevel, false, false, false, retryAtTicks, retryBackoffTicks};
  train.navigation.state = marklin::NavigationSystem::State::FindingPath;
}

inline void resetFindingPathRetryBackoff(marklin::Train& train) {
  train.navigation.findingPathTask.retryAtTicks = 0;
  train.navigation.findingPathTask.retryBackoffTicks = 0;
}

inline void scheduleFindingPathRetry(TrainTrackServerContext& context, marklin::Train& train) {
  auto& task = train.navigation.findingPathTask;
  task.retryBackoffTicks = task.retryBackoffTicks == 0
                               ? FINDING_PATH_RETRY_INITIAL_TICKS
                               : kit::min(task.retryBackoffTicks * 2, FINDING_PATH_RETRY_MAX_TICKS);
  task.retryAtTicks = context.currentTicks + task.retryBackoffTicks;
}

inline void setSpeedCmdHandler(TrainTrackServerContext& context, marklin::TrainId trainId,
                               marklin::SpeedLevel speedLevel) {
  KIT_ASSERT(marklin::isValidTrainId(trainId), "invalid train id");
  KIT_ASSERT(marklin::isValidSpeedLevel(speedLevel), "invalid train speed level");

  // Check busy status.
  initTrain(context, trainId);
  marklin::Train& train = context.ttState.getTrain(trainId);
  if (train.navigation.state == marklin::NavigationSystem::State::Reversing) {
    train.navigation.reversingTask.preReversingSpeedLevel = speedLevel;
    notifyStatusToUI(context.uiTid, "Train %u will reverse with speed %u.", trainId, speedLevel);
    return;
  }
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
  if (speedLevel == 0) {
    train.navigation.isWandering = false;
    notifyStatusToUI(context.uiTid, "Train %u stopped wandering.", trainId);
    return;
  }

  if (train.navigation.state != marklin::NavigationSystem::State::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", trainId);
    return;
  }

  train.navigation.isWandering = true;
  makeTrainGotoRandomSensor(context, train, speedLevel);
  broadcastTrainSpeedLevel(context, trainId, speedLevel);
  notifyStatusToUI(context.uiTid, "Train %u wandering at speed %u.", trainId, speedLevel);
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

inline void toggleSwitchCmdHandler(TrainTrackServerContext& context, marklin::SwitchId switchId) {
  KIT_ASSERT(marklin::isValidSwitchId(switchId), "invalid switch id");
  marklin::SwitchState nextState = context.ttState.getSwitchState(switchId) == marklin::SwitchState::Straight
                                       ? marklin::SwitchState::Curved
                                       : marklin::SwitchState::Straight;
  broadcastSwitchState(context, switchId, nextState);
}

inline void setTrackCmdHandler(TrainTrackServerContext& context, marklin::TrackId trackId) {
  KIT_ASSERT(marklin::isValidTrack(trackId), "invalid track id");
  resetContext(context);
  context.ttState.setCurrentTrack(trackId);
  notifyStatusToUI(context.uiTid, "Switched track to %c.", (trackId == 0 ? 'A' : 'B'));
}

inline uint32_t absTickDelta(uint32_t lhs, uint32_t rhs) { return lhs >= rhs ? lhs - rhs : rhs - lhs; }

struct SensorAttributionCandidate {
  marklin::TrainId trainId;
  int32_t score;
};

inline SensorAttributionCandidate scoreSensorCandidate(TrainTrackServerContext& context, marklin::TrackNode& sensor,
                                                       marklin::TrainId trainId, marklin::TrainId reserverId) {
  SensorAttributionCandidate candidate{.trainId = trainId, .score = 0};
  marklin::Train& train = context.ttState.getTrain(trainId);

  if (reserverId == trainId) {
    candidate.score += 100;
  } else if (reserverId != marklin::NO_TRAIN && train.navigation.state == marklin::NavigationSystem::State::Routed) {
    candidate.score -= 50;
  }

  if (train.prediction.sensor && train.prediction.sensor->id == sensor.id) {
    candidate.score += 200;
  }

  if (train.kinematics.state == marklin::KinematicsSystem::State::Tracked && train.kinematics.estimatedNode) {
    marklin::Distance distToSensor =
        marklin::shortestDistanceToNode(context.ttState, train.kinematics.estimatedNode->id, sensor.id) -
        train.kinematics.estimatedNodeOffset;
    if (distToSensor < marklin::INF_DISTANCE) {
      int32_t distanceScore = 220 - distToSensor / 15'000;
      candidate.score += kit::clamp(distanceScore, int32_t(-90), int32_t(220));
    } else {
      candidate.score -= 1000;
    }
  } else {
    candidate.score -= 30;
  }

  if (train.navigation.state == marklin::NavigationSystem::State::Manual) {
    candidate.score += 50;
  }

  return candidate;
}

inline marklin::TrainId attributeSensorOwner(TrainTrackServerContext& context, marklin::TrackNode& sensor) {
  marklin::TrainId reserverId = context.pfSystem.getReserver(sensor.id);
  SensorAttributionCandidate best{.trainId = marklin::NO_TRAIN, .score = std::numeric_limits<int32_t>::min()};
  for (marklin::TrainId id : context.activeTrains) {
    SensorAttributionCandidate candidate = scoreSensorCandidate(context, sensor, id, reserverId);
    if (candidate.score > best.score) {
      best = candidate;
    }
  }
  return best.trainId;
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

  marklin::TrainId ownerId = attributeSensorOwner(context, sensor);
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

  initTrain(context, id);
  marklin::Train& train = context.ttState.getTrain(id);
  const bool wasManual = train.navigation.state == marklin::NavigationSystem::State::Manual;
  if (train.navigation.state == marklin::NavigationSystem::State::Reversing) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", id);
    return;
  }

  train.navigation.isWandering = false;
  train.navigation.findingPathTask = {dest->id, offset, speedLevel, false, false, 0, 0};
  train.navigation.oldPathingState = marklin::PathingState::Idling;
  train.navigation.state = marklin::NavigationSystem::State::FindingPath;
  broadcastTrainSpeedLevel(context, id, speedLevel);
  notifyStatusToUI(context.uiTid, "Goto train %u to %s%s.", id, dest->name, wasManual ? "" : " (retarget)");
}

inline void timerTickHandler(TrainTrackServerContext& context, uint32_t ticks) {
  context.currentTicks = ticks;
  bool shouldUpdateTrainUI = context.currentTicks - context.lastTrainUIRefreshTicks >= 10;
  bool shouldSendGameStateToPacman = context.currentTicks - context.lastPacmanRefreshTicks >= 50;

  if (!context.hasEmergency && context.pfSystem.hasEmergency()) {
    context.hasEmergency = true;
  } else if (context.hasEmergency && !context.pfSystem.hasEmergency()) {
    context.hasEmergency = false;
    context.lastEmergencyTick = ticks;
  }

  for (marklin::TrainId trainId : context.activeTrains) {
    marklin::Train& train = context.ttState.getTrain(trainId);

    // Part A: Kinematics Block.
    train.kinematics.onTick(context.ttState, trainId);

    // Part B: Navigation Block.
    switch (train.navigation.state) {
    case marklin::NavigationSystem::State::Manual: {
      break;
    }
    case marklin::NavigationSystem::State::FindingPath: {
      // Train must be tracked.
      if (train.kinematics.state != marklin::KinematicsSystem::State::Tracked) {
        break;
      }

      if (train.navigation.findingPathTask.retryAtTicks > context.currentTicks) {
        break;
      }

      // Check if the train is reversing.
      bool wasReversing = false;
      if (train.navigation.findingPathTask.isReversing) {
        // Wait for the reverse to complete.
        if (!train.kinematics.isStationary()) {
          break;
        }
        // Update kinematics and prediction.
        broadcastReverseTrainDirection(context, trainId);
        train.kinematics.reverseSensor();
        marklin::Distance distToNext = 0;
        marklin::TrackNode* nextSensor =
            marklin::getNextSensor(context.ttState, *train.kinematics.lastSensor, distToNext);
        train.prediction.triggerSensor(*train.kinematics.lastSensor, nextSensor, distToNext,
                                       train.kinematics.estimatedSpeed, context.currentTicks);
        train.navigation.findingPathTask.isReversing = false;
        wasReversing = true;
      }

      // Find path from the current direction. If fail, we try the other direction.
      if (context.pfSystem.planPath(context.ttState, trainId, train.kinematics.estimatedNode->id,
                                    train.navigation.findingPathTask.dest, train.navigation.findingPathTask.offset)) {
        resetFindingPathRetryBackoff(train);
        train.navigation.state = marklin::NavigationSystem::State::Routed;
        notifyStatusToUI(context.uiTid, "Train %u routed to %s.", trainId,
                         context.ttState.getTrackNodeById(train.navigation.findingPathTask.dest).name);
      } else {
        if (wasReversing) {
          // Already tried reversing and still couldn't find a path.
          // Most likekly blocked by other train. Pick a new destination.
          const marklin::TrackNodeId failedDest = train.navigation.findingPathTask.dest;
          scheduleFindingPathRetry(context, train);
          makeTrainGotoRandomSensor(context, train, train.navigation.findingPathTask.maxSpeedLevel);
          notifyStatusToUI(context.uiTid, "Train %u failed to find a path from %s to %s.", trainId,
                           train.kinematics.estimatedNode->name, context.ttState.getTrackNodeById(failedDest).name);
        } else {
          // Try the other direction.
          train.navigation.findingPathTask.isReversing = true;
          broadcastTrainSpeedLevel(context, trainId, 0);
        }
      }
      break;
    }
    case marklin::NavigationSystem::State::Routed: {
      auto pathFindingState = context.pfSystem.updateState(
          trainId, train, [&](marklin::SwitchId id, marklin::SwitchState st) { broadcastSwitchState(context, id, st); },
          [&]<class... Args>(kit::FormatSpec<Args...> fmt, const Args&... args) {
            notifyStatusToUI(context.uiTid, fmt, args...);
          });
      const bool changed = pathFindingState != train.navigation.oldPathingState;

      switch (pathFindingState) {
      case marklin::PathingState::Idling: {
        // Train has no path finding task. Stationary.
        if (changed) {
          notifyStatusToUI(context.uiTid, "Train %u arrived at %s", trainId,
                           context.ttState.getTrackNodeById(train.navigation.findingPathTask.dest).name);
        }
        if (train.navigation.isWandering) {
          makeTrainGotoRandomSensor(context, train, train.navigation.findingPathTask.maxSpeedLevel);
        } else {
          train.navigation.state = marklin::NavigationSystem::State::Manual;
        }
        break;
      }
      case marklin::PathingState::Moving: {
        // Can enter all nodes within the stopping distance.
        // Moving toward the destination.
        if (changed) {
          broadcastTrainSpeedLevel(context, trainId, train.navigation.findingPathTask.maxSpeedLevel);
        }
        break;
      }
      case marklin::PathingState::Yielding: {
        // Can not enter all nodes within the stopping distance.
        // Slowing down.
        if (changed) {
          broadcastTrainSpeedLevel(context, trainId, 0);
        }
        break;
      }
      case marklin::PathingState::Arriving: {
        // Can enter all nodes within the stopping distance.
        // Destination is within the stopping distance.
        // Slowing down.
        if (changed) {
          broadcastTrainSpeedLevel(context, trainId, 0);
        }
        break;
      }
      case marklin::PathingState::Trespassing: {
        if (changed) {
          broadcastTrainSpeedLevel(context, trainId, 0);
        }
        break;
      }
      case marklin::PathingState::Resuming: {
        if (train.navigation.findingPathTask.reqeusetToResume) {

          if (ticks - context.lastEmergencyTick < 100) {
            train.navigation.findingPathTask.isResumed = false;
            notifyStatusToUI(context.uiTid, "Train %u abort resuming", trainId);
            break;
          }

          bool isResumed =
              context.pfSystem.planPath(context.ttState, trainId, train.kinematics.estimatedNode->id,
                                        train.navigation.findingPathTask.dest, train.navigation.findingPathTask.offset);
          if (!isResumed) {
            // Reverse and try agian.
            // Update kinematics and prediction.
            broadcastReverseTrainDirection(context, trainId);
            train.kinematics.reverseSensor();
            marklin::Distance distToNext = 0;
            marklin::TrackNode* nextSensor =
                marklin::getNextSensor(context.ttState, *train.kinematics.lastSensor, distToNext);
            train.prediction.triggerSensor(*train.kinematics.lastSensor, nextSensor, distToNext,
                                           train.kinematics.estimatedSpeed, context.currentTicks);
            isResumed = context.pfSystem.planPath(context.ttState, trainId, train.kinematics.estimatedNode->id,
                                                  train.navigation.findingPathTask.dest,
                                                  train.navigation.findingPathTask.offset);
          }

          train.navigation.findingPathTask.isResumed = isResumed;
          if (isResumed) {
            notifyStatusToUI(context.uiTid, "Train %u resumed to the destination %s", trainId,
                             context.ttState.getTrackNodeById(train.navigation.findingPathTask.dest).name);
          } else {
            notifyStatusToUI(context.uiTid, "Train %u failed to resume to the destination %s", trainId,
                             context.ttState.getTrackNodeById(train.navigation.findingPathTask.dest).name);
          }
        }
        break;
      }
      }

      train.navigation.oldPathingState = pathFindingState;
      break;
    }
    case marklin::NavigationSystem::State::Reversing: {
      if (train.kinematics.isStationary()) {
        broadcastReverseTrainDirection(context, trainId);
        if (train.kinematics.state == marklin::KinematicsSystem::State::Tracked) {
          train.kinematics.reverseSensor();
          marklin::Distance distToNext = 0;
          marklin::TrackNode* nextSensor =
              marklin::getNextSensor(context.ttState, *train.kinematics.lastSensor, distToNext);
          train.prediction.triggerSensor(*train.kinematics.lastSensor, nextSensor, distToNext,
                                         train.kinematics.estimatedSpeed, context.currentTicks);
          broadcastTrainSpeedLevel(context, trainId, train.navigation.reversingTask.preReversingSpeedLevel);
        }
        train.navigation.state = marklin::NavigationSystem::State::Manual;
      }
      break;
    }
    }

    // Log to UI.
    if (shouldUpdateTrainUI) {
      KIT_ASSERT(!context.trainStates.full(), "train state buffer overflow");
      bool lockEnd = false;
      TrainStatesEntry entry{.trainId = trainId, .train = &train, .nodes = {}, .nodeCount = 0, .lockCount = 0};
      for (auto& node : context.pfSystem.getTrainPath(trainId).nodes) {
        if (!lockEnd && context.pfSystem.getReserver(node.srce->id) == trainId) {
          ++entry.lockCount;
        } else {
          lockEnd = true;
        }
        entry.nodes[entry.nodeCount++] = &context.ttState.getTrackNodeById(node.srce->id);
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
  if (shouldSendGameStateToPacman) {
    context.lastPacmanRefreshTicks = context.currentTicks;
    sendGameStateToPacman(context);
  }
}

} // namespace k4
