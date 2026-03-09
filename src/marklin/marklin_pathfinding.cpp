#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "user_tasks/send_util.h"
#include "util/debug.h"
#include "util/kit_algorithm.h"
#include "util/ring_buffer.h"
#include "util/static_priority_queue.h"
#include <limits>

namespace marklin {

namespace {
constexpr int32_t EWMA_DENOMINATOR = 4;
constexpr std::array STOPPING_DISTANCE = {0,     2100,  3600,  5600,  9500,  13100,  19800, 26400,
                                          37800, 42700, 57900, 76600, 98000, 118400, 152300};

constexpr marklin::TrackNodeId loopSensorNodeIds[] = {
    marklin::sensorToTrackNodeId({'A', 3}),  marklin::sensorToTrackNodeId({'A', 4}),
    marklin::sensorToTrackNodeId({'B', 15}), marklin::sensorToTrackNodeId({'B', 16}),
    marklin::sensorToTrackNodeId({'C', 9}),  marklin::sensorToTrackNodeId({'C', 10}),
    marklin::sensorToTrackNodeId({'B', 1}),  marklin::sensorToTrackNodeId({'B', 2}),
    marklin::sensorToTrackNodeId({'D', 13}), marklin::sensorToTrackNodeId({'D', 14}),
    marklin::sensorToTrackNodeId({'E', 13}), marklin::sensorToTrackNodeId({'E', 14}),
    marklin::sensorToTrackNodeId({'E', 9}),  marklin::sensorToTrackNodeId({'E', 10}),
    marklin::sensorToTrackNodeId({'D', 5}),  marklin::sensorToTrackNodeId({'D', 6}),
    marklin::sensorToTrackNodeId({'E', 5}),  marklin::sensorToTrackNodeId({'E', 6}),
    marklin::sensorToTrackNodeId({'D', 3}),  marklin::sensorToTrackNodeId({'D', 4}),
    marklin::sensorToTrackNodeId({'B', 5}),  marklin::sensorToTrackNodeId({'B', 6}),
    marklin::sensorToTrackNodeId({'C', 11}), marklin::sensorToTrackNodeId({'C', 12}),
};

// The node is NOT passable if:
// A different train sitting on the node on either direction
// The same train sittong on the same direction.
bool isPassable(TrackNode& node, TrainId id) {
  if (node.owner != NO_TRAIN && node.owner != id) {
    return false;
  }
  if (node.reverse->owner != NO_TRAIN && node.reverse->owner != id) {
    return false;
  }
  if (node.owner == id) {
    return false;
  }
  return true;
}

// Run dijkstra to find the shortest AND viable path between srce and dest.
bool dijkstra(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest, Distance& outTotalPathDist) {
  // Already at the destination.
  Train& train = ttState.getTrain(trainId);
  train.path.clear();
  outTotalPathDist = 0;
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
  struct Edge {
    Distance dist;
    TrackNodeId id;
    auto operator<=>(const Edge&) const = default;
  };
  StaticPriorityQueue<Edge, NUM_TRACK_NODES * 2> queue;
  queue.push({0, srce});

  bool isReachable = false;
  while (!queue.empty()) {
    Edge u = queue.top();
    queue.pop();

    if (u.id == dest) {
      isReachable = true;
      break;
    }

    if (u.dist > minDistances[u.id]) {
      continue;
    }

    // Calculate number of edges.
    const TrackNode& uNode = ttState.getTrackNodeById(u.id);
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
      if (!isPassable(*edge.dest, trainId)) {
        continue;
      }

      // Update distance and parent.
      TrackNodeId v = edge.dest->id;
      Distance dist = u.dist + edge.dist;
      if (dist < minDistances[v]) {
        minDistances[v] = dist;
        parents[v] = u.id;
        queue.push({dist, v});
      }
    }
  }

  // Extract the path.
  if (isReachable) {
    for (TrackNodeId u = dest; u != srce; u = parents[u]) {
      TrackNode* node = &ttState.getTrackNodeById(u);
      node->owner = trainId;
      train.path.pushFront(node);
      outTotalPathDist += getAdjacentDistance(ttState.getTrackNodeById(parents[u]), *node);
    }
  }

  return isReachable;
}
} // namespace

Distance getStoppingDistance(SpeedLevel speedLevel) { return STOPPING_DISTANCE[speedLevel]; }

bool lockAllLoopSensorNodes(TrainTrackState& ttState, TrainId trainId) {
  for (auto id : loopSensorNodeIds) {
    if (ttState.getTrackNodeById(id).owner != NO_TRAIN) {
      return false;
    }
  }
  for (auto id : loopSensorNodeIds) {
    ttState.getTrackNodeById(id).owner = trainId;
  }
  return true;
}

void unlockAllLoopSensorNodes(TrainTrackState& ttState) {
  for (auto id : loopSensorNodeIds) {
    ttState.getTrackNodeById(id).owner = NO_TRAIN;
  }
}

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

Distance getAdjacentDistance(TrackNode& n1, TrackNode& n2) {
  switch (n1.type) {
  case TrackNode::Type::Branch:
    if (TrackNode* u = n1.edges[0].dest; u && u->id == n2.id) {
      return n1.edges[0].dist;
    }
    if (TrackNode* u = n1.edges[1].dest; u && u->id == n2.id) {
      return n1.edges[1].dist;
    }
    logError("n1 and n2 are not adjacent");
  case TrackNode::Type::Sensor:
  case TrackNode::Type::Merge:
  case TrackNode::Type::Enter:
    if (TrackNode* u = n1.edges[0].dest; u && u->id == n2.id) {
      return n1.edges[0].dist;
    }
    logError("n1 and n2 are not adjacent");
  default:
    logError("n1 and n2 are not adjacent");
  }
}

bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId dest, Distance& outTotalPathDist) {
  // Calculate the shortest and viable path.
  bool isReachable = dijkstra(ttState, trainId, dest, outTotalPathDist);
  if (!isReachable) {
    return false;
  }

  // Acquire the lock.

  // Calcualte the track node to stop.

  // Lock all the path.

  return isReachable;
}

void calibrateTrain(TrainTrackState& ttState, TrackNode& triggeredNode, uint32_t currentTicks) {
  Train& train = ttState.getTrain(triggeredNode.owner);

  // Update the train estimated speed.
  uint32_t dT = kit::max(1u, currentTicks - train.lastSpeedUpdateTicks);
  Distance dS = [&] -> Distance {
    // The train must be located first.
    TrackNode* last = train.lastVisitedNode;
    if (!last) {
      return 0;
    }

    // Remove all the missing nodes and sum up the distance.
    last->owner = NO_TRAIN;
    Distance totalDist = 0;
    while (!train.path.empty()) {
      TrackNode* curr = train.path.popFront();
      curr->owner = NO_TRAIN;
      totalDist += getAdjacentDistance(*last, *curr);
      last = curr;
      if (curr->id == triggeredNode.id) {
        break;
      }
    }
    return totalDist;
  }();

  // Update the time weighted speed.
  if (dS != 0) {
    Speed v = dS / Speed(dT);
    if (train.estimatedSpeed == 0) {
      train.estimatedSpeed = v;
    } else {
      train.estimatedSpeed = (train.estimatedSpeed * (EWMA_DENOMINATOR - 1) + v) / EWMA_DENOMINATOR;
    }
    train.stateMachine.pathing.pathDistance -= dS;
  }

  // Update the train last triggered sensor node.
  train.estimatedNode = &triggeredNode;
  train.estimatedOffset = 0;
  train.lastVisitedNode = &triggeredNode;
  train.lastSpeedUpdateTicks = currentTicks;
}
} // namespace marklin
