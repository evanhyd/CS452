#pragma once
#include "marklin_train_track.h"
#include <array>

namespace marklin {
// Get the stopping distance based on the speed.
Distance getStoppingDistance(Speed speed);

// Convert speed level to offline data speed.
Speed convertSpeedLevelToOfflineSpeed(SpeedLevel speedLevel);

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

// Path finding system. Responsible for plan and reserve a path for the train.
// Reserving a path is different from locking a path.
// Reserving ensures the train path won't lead to unavoidable collisions.
// Locking ensures sole access when the train is physically passing.
class PathFindingSystem {
  struct Entry {
    TrainId id;
    int time;
  };
  std::array<RingBuffer<Entry, NUM_TRAIN_IN_LAB>, NUM_TRACK_NODES> reservations_{};
  std::array<Entry, NUM_TRACK_NODES> locks_;
  std::array<RingBuffer<TrackNodeId, MAX_NODE_PER_TRAIN>, MAX_TRAIN_ID> ownedNodes_;

  bool dijkstra(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest, Distance& outTotalPathDist);

public:
  PathFindingSystem();

  void reset();

  // Reserve a path for train to go to the destination.
  // Return true if a viable path is reserved.
  bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest, Distance& outTotalPathDist);

  // Reservation
  void reserve(TrackNodeId nodeId, TrainId trainId);
  void unreserve(TrackNodeId nodeId, TrainId trainId, std::source_location loc = std::source_location::current());
  bool canReserve(const TrackNode& node, TrainId trainId) const;

  // Locking
  void lock(TrackNodeId nodeId, TrainId trainId, std::source_location loc = std::source_location::current());
  void unlock(TrackNodeId nodeId, TrainId trainId, std::source_location loc = std::source_location::current());
  bool canLock(TrackNodeId nodeId, TrainId trainId) const;
  TrainId getOwner(TrackNodeId nodeId) const;
  const RingBuffer<TrackNodeId, MAX_NODE_PER_TRAIN>& getOwned(TrainId trainId) const;
};

} // namespace marklin
