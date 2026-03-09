#pragma once
#include "marklin_train_track.h"

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

// Estimate and update the train's motion data.
void calibrateTrain(Train& train, TrackNode& triggeredNode, uint32_t currentTicks, Distance dS);

template <typename OnRelease>
void calibrateTrainAndSetSwitches(TrainTrackState& ttState, TrackNode& triggeredNode, uint32_t currentTicks,
                                  const OnRelease& onRelease) {
  TrainId trainId = triggeredNode.lock.owner();
  Train& train = ttState.getTrain(trainId);

  Distance dS = [&] -> Distance {
    // The train must be located first.
    TrackNode* last = train.lastVisitedNode;
    if (!last) {
      return 0;
    }

    // Remove all the missing nodes and sum up the distance.
    last->lock.release(trainId);
    onRelease(*last);
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

  calibrateTrain(train, triggeredNode, currentTicks, dS);
}

} // namespace marklin
