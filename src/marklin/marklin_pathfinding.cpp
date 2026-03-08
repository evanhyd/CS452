#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "user_tasks/send_util.h"
#include "util/kit_algorithm.h"
#include "util/ring_buffer.h"
#include "util/static_priority_queue.h"
#include <limits>

namespace marklin {

namespace {
constexpr int32_t EWMA_DENOMINATOR = 4;
constexpr std::array STOP_DISTANCE = {0,     2100,  3600,  5600,  9500,  13100,  19800, 26400,
                                      37800, 42700, 57900, 76600, 98000, 118400, 152300};

// Return true if the track node is part of the loop.
bool isNodeInLoop(TrackNodeId id) {
  static constexpr std::array loopIds = {
      marklin::sensorToTrackNodeId({'D', 1}),  marklin::sensorToTrackNodeId({'D', 2}),
      marklin::sensorToTrackNodeId({'E', 3}),  marklin::sensorToTrackNodeId({'E', 4}),
      marklin::sensorToTrackNodeId({'E', 5}),  marklin::sensorToTrackNodeId({'E', 6}),
      marklin::sensorToTrackNodeId({'D', 5}),  marklin::sensorToTrackNodeId({'D', 6}),
      marklin::sensorToTrackNodeId({'E', 9}),  marklin::sensorToTrackNodeId({'E', 10}),
      marklin::sensorToTrackNodeId({'E', 13}), marklin::sensorToTrackNodeId({'E', 14}),
      marklin::sensorToTrackNodeId({'D', 15}), marklin::sensorToTrackNodeId({'D', 16}),
      marklin::sensorToTrackNodeId({'B', 13}), marklin::sensorToTrackNodeId({'B', 14}),
  };
  return kit::contains(loopIds.begin(), loopIds.end(), id);
}

// Run dijkstra to find the shortest AND viable path between srce and dest.
bool dijkstra(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest) {
  Train& train = ttState.getTrain(trainId);

  // Already at the destination.
  TrackNodeId srce = train.lastVisitedNode->id;
  if (srce == dest) {
    return true;
  }

  // Track min distances.
  static constexpr Distance INFINITY = std::numeric_limits<Distance>::max();
  std::array<Distance, NUM_TRACK_NODES> minDistances;
  kit::fill(minDistances.begin(), minDistances.end(), INFINITY);
  minDistances[srce] = 0;

  // Track parents.
  static constexpr TrackNodeId NO_PARENT = NUM_TRACK_NODES;
  std::array<TrackNodeId, NUM_TRACK_NODES> parents;
  kit::fill(parents.begin(), parents.end(), NO_PARENT);

  // Dijkstrak go brrrrrr.
  StaticPriorityQueue<TrackNodeId, NUM_TRACK_NODES> queue;
  queue.push(srce);

  bool isReachable = false;
  while (!isReachable && !queue.empty()) {
    TrackNodeId u = queue.top();
    queue.pop();
    const TrackNode& uNode = ttState.getTrackNodeById(u);

    size_t numEdges = [&] -> size_t {
      switch (uNode.type) {
      case TrackNode::Type::Branch:
        return 2;
      case TrackNode::Type::Sensor:
      case TrackNode::Type::Merge:
      case TrackNode::Type::Enter:
        return 1;
      default:
        return 0;
      }
    }();

    // Explore neighbors.
    for (size_t i = 0; i < numEdges; ++i) {
      const TrackEdge& edge = uNode.edges[i];
      // CUSTOM RULE: Check existing track node ownership.
      if (edge.dest->trainOwnerId != 0 && edge.dest->trainOwnerId != trainId) {
        continue;
      }

      // Update distance and parent.
      TrackNodeId v = edge.dest->id;
      Distance dist = minDistances[u] + edge.dist;
      if (dist < minDistances[v]) {
        minDistances[v] = dist;
        parents[v] = u;
        if (v == dest) {
          isReachable = true;
          break;
        }
        queue.push((v));
      }
    }
  }

  // Extract the path.
  if (isReachable) {
    train.path.clear();
    for (TrackNodeId u = dest; u != srce && u != NO_PARENT; u = parents[u]) {
      TrackNode* node = &ttState.getTrackNodeById(u);
      node->trainOwnerId = trainId;
      train.path.pushFront(node);
    }
  }

  return isReachable;
}
} // namespace

TrackNode* getNextTrackNode(TrainTrackState& state, TrackNode& srce, Distance& outDistance) {
  if (srce.type == TrackNode::Type::Exit) {
    return nullptr;
  }
  size_t dir = TrackDirection::Ahead;
  if (srce.type == TrackNode::Type::Branch) {
    auto sw = state.getSwitchState(srce.num);
    dir = (sw == SwitchState::Straight ? TrackDirection::Straight : TrackDirection::Curved);
  }
  outDistance = srce.edges[dir].dist;
  return srce.edges[dir].dest;
}

TrackNode* getNextSensor(TrainTrackState& state, TrackNode& srce, Distance& outDistance) {
  outDistance = 0;
  TrackNode* node = &srce;
  while (true) {
    Distance dist = 0;
    node = getNextTrackNode(state, *node, dist);
    outDistance += dist;
    if (!node || node->type == TrackNode::Type::Sensor) {
      break;
    }
  }
  return node;
}

bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest) {
  // Calculate the shortest and viable path.
  bool isReachable = dijkstra(ttState, trainId, dest);
  if (!isReachable) {
    return false;
  }

  // Acquire the lock.

  // Calcualte the track node to stop.

  // Lock all the path.

  return isReachable;
}

void calibrateTrainFromTrackNode(TrainTrackState& ttState, TrackNodeId triggeredTrackId, uint32_t currentTicks) {
  // TODO: take network delay into consideration.
  TrackNode& triggeredSensor = ttState.getTrackNodeById(triggeredTrackId);
  if (triggeredSensor.trainOwnerId == 0) {
    return;
  }

  Train& train = ttState.getTrain(triggeredSensor.trainOwnerId);

  // Update the train estimated speed.
  uint32_t dT = kit::max(1u, currentTicks - train.lastVisitedTicks);
  Distance dS = [&] -> Distance {
    // If the train is not located, then skip the speed calculation.
    TrackNode* lastSensor = train.lastVisitedNode;
    if (!lastSensor) {
      return 0;
    }

    // Calculate the speed. Need multiple hop because the train can miss the sensor.
    Distance totalDistance = train.estimatedOffset;
    static constexpr int MAX_HOPS = 20;
    for (int hop = 0; hop < MAX_HOPS; ++hop) {
      Distance dist = 0;
      lastSensor = getNextSensor(ttState, *lastSensor, dist);
      if (!lastSensor) {
        return 0;
      }

      totalDistance += dist;
      if (lastSensor->id == triggeredTrackId) {
        return totalDistance;
      }
    }
    return 0;
  }();

  // Update the time weighted speed.
  if (dS != 0) {
    Speed v = dS / Speed(dT);
    train.estimatedSpeed = (train.estimatedSpeed * (EWMA_DENOMINATOR - 1) + v) / EWMA_DENOMINATOR;
  }

  // Update the train last triggered sensor node.
  train.lastVisitedNode = &triggeredSensor;
  train.lastVisitedTicks = currentTicks;
  train.estimatedOffset = 0;
}
} // namespace marklin
