#pragma once
#include "marklin_train_track.h"
#include "util/debug.h"
#include "util/kit_algorithm.h"

namespace marklin {

// Get the stopping distance based on the speed level.
Distance getStoppingDistance(SpeedLevel speedLevel);

// Lock all the sensors on the loop.
bool lockAllLoopSensorNodes(TrainTrackState& ttState, TrainId trainId);

// Unlock all the sensors on the loop.
void unlockAllLoopSensorNodes(TrainTrackState& ttState, TrainId trainId);

// Get the next track node by following the track at the current configuration.
// Return nullptr if not found.
TrackNode* getNextTrackNode(TrainTrackState& state, TrackNode& srce, Distance& outDistance);

// Get the next sensor track node by following the track at the current configuration.
// Return nullptr if not found.
TrackNode* getNextSensor(TrainTrackState& state, TrackNode& srce, Distance& outDistance);

// Get the distance from n1 to n2.
// n1 and n2 must be adjacent to each other.
Distance getAdjacentDistance(TrackNode& n1, TrackNode& n2);

// Get the direction from n1 to n2.
// n1 and n2 must be adjacent to each other.
TrackDirection getAdjacentDirection(TrackNode& n1, TrackNode& n2);

// Plan a path for train to go to the destination.
// The train must acquire the ownership of the path.
// Return true if a viable path is acquired.
bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest, Distance& outTotalPathDist);

template <typename OnRelease>
void calibrateTrainAndSetSwitches(TrainTrackState& ttState, TrackNode& triggeredNode, uint32_t currentTicks,
                                  const OnRelease& onRelease) {
  TrainId trainId = triggeredNode.lock.owner();
  Train& train = ttState.getTrain(trainId);
  if (train.stateMachine.type != TrainStateMachine::Type::Locating ||
      train.stateMachine.type != TrainStateMachine::Type::Pathing) {
    logError("calibrated a train that's not in locating or pathing state.");
  }

  // Only calculate the train motion if already calibrated (Pathing).
  if (train.stateMachine.type == TrainStateMachine::Type::Pathing) {
    if (!train.lastVisitedNode) {
      logError("last visited node is null");
    }

    Distance dS = [&] -> Distance {
      // First node guarantee outdated.
      TrackNode* last = train.lastVisitedNode;
      last->lock.release(trainId);
      onRelease(*last);

      // Remove all the missing nodes and sum up the distance.
      Distance totalDist = 0;
      while (!train.path.empty()) {
        TrackNode* curr = train.path.popFront();
        totalDist += getAdjacentDistance(*last, *curr);
        if (curr->id == triggeredNode.id) {
          break;
        }
        curr->lock.release(trainId);
        onRelease(*curr);
        last = curr;
      }
      return totalDist;
    }();

    // Update the train estimated speed.
    uint32_t dT = kit::max(1u, currentTicks - train.lastCalibrateTicks);

    // Update the time weighted speed.
    Speed v = dS / Speed(dT);
    if (train.estimatedSpeed == 0) {
      train.estimatedSpeed = v;
    } else {
      constexpr int32_t EWMA_DENOMINATOR = 4;
      train.estimatedSpeed = (train.estimatedSpeed * (EWMA_DENOMINATOR - 1) + v) / EWMA_DENOMINATOR;
    }
    train.stateMachine.pathing.pathDistance -= dS;
    train.estimatedPathDistance = train.stateMachine.pathing.pathDistance;
  }

  // Update the train last triggered sensor node.
  train.lastCalibrateTicks = currentTicks;
  train.lastVisitedNode = &triggeredNode;
  train.estimatedOffsetFromLast = 0;
  train.estimatedNode = &triggeredNode;
  train.estimatedOffsetFromEstimatedNode = 0;
}

} // namespace marklin
