#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "util/debug.h"
#include "util/ring_buffer.h"
#include "util/static_priority_queue.h"
#include <limits>

namespace marklin {

namespace {
static constexpr int32_t EWMA_DENOMINATOR = 4;

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
bool dijkstra(TrainTrackState& ttState, TrainId trainId, TrackNodeId srce, TrackNodeId dest) {
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
    const TrackNode& uNode = ttState.getTrackNodeRef(u);

    size_t numEdges = [&uNode] -> size_t {
      switch (uNode.type) {
      case TrackNode::Type::Sensor:
      case TrackNode::Type::Merge:
      case TrackNode::Type::Enter:
        return 2;
      case TrackNode::Type::Branch:
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
    auto& path = ttState.getTrainRef(trainId).path;
    path.clear();
    for (TrackNodeId u = dest; u != srce; u = parents[u]) {
      path.pushFront(u);
    }
  }

  return isReachable;
}
} // namespace

TrackNode* getNextSensor(TrainTrackState& state, TrackNode& srce, Distance& outDistance) {
  TrackNode* node = &srce;
  outDistance = 0;
  while (node && node->type != TrackNode::Type::Sensor) {
    if (node->type == TrackNode::Type::Exit) {
      return nullptr;
    }
    size_t dir = TrackDirection::Ahead;
    if (node->type == TrackNode::Type::Branch) {
      auto sw = state.getSwitchState(node->num);
      dir = (sw == SwitchState::Straight ? TrackDirection::Straight : TrackDirection::Curved);
    }
    outDistance += node->edges[dir].dist;
    node = node->edges[dir].dest;
  }
  return node;
}

bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId destId) {
  Train& train = ttState.getTrainRef(trainId);

  // Calculate the shortest and viable path.
  bool isReachable = dijkstra(ttState, trainId, train.lastVisitedNodeId, destId);

  // Enrich with the loop.

  // Calcualte the track node to stop.

  // Lock all the path.

  return isReachable;
}

void calibrateTrainFromTrackNode(TrainTrackState& ttState, TrackNodeId triggeredTrackId, uint32_t currentTicks) {
  // TODO: replace currentTicks with estimated network event tick
  // TODO: assign owner during path planning.
  TrackNode& triggeredSensor = ttState.getTrackNodeRef(triggeredTrackId);
  triggeredSensor.trainOwnerId = ttState.theTrain;
  if (triggeredSensor.trainOwnerId == 0) {
    return;
  }

  Train& train = ttState.getTrainRef(triggeredSensor.trainOwnerId);

  // Update the train estimated speed.
  uint32_t dT = kit::max(1u, currentTicks - train.lastVisitedTicks);
  Distance dS = [&] -> Distance {
    Distance totalDistance = train.estimatedOffset;
    TrackNode* lastSensor = &ttState.getTrackNodeRef(train.lastVisitedNodeId);
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
  train.lastVisitedNodeId = triggeredSensor.id;
  train.lastVisitedTicks = currentTicks;
  train.estimatedOffset = 0;
}
} // namespace marklin
