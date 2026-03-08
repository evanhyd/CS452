#pragma once
#include "marklin_train_track.h"

namespace marklin {
// Get the next sensor track node by following the track at the current configuration.
// Return nullptr if not found.
TrackNode* getNextSensor(TrainTrackState& state, TrackNode& srce, Distance& outDistance);

// Plan a path for train to go to the destination.
// The train must acquire the ownership of the path.
// Return true if a viable path is acquired.
bool planPath(TrainTrackState& ttState, TrainId trainId, TrackNodeId destId);

// Estimate and update the train's motion data.
void calibrateTrainFromTrackNode(TrainTrackState& ttState, TrackNodeId triggeredTrackId, uint32_t currentTicks);

// TrackNodeId findNodeIdByName(TrainTrackState& ttState, const char* name) {
//   const auto& track = ttState.getCurrentTrack();
//   for (size_t i = 0; i < marklin::NUM_TRACK_NODES; ++i) {
//     if (track[i].name == nullptr)
//       continue;
//     const char* a = track[i].name;
//     const char* b = name;
//     while (*a && *b && *a == *b) {
//       ++a;
//       ++b;
//     }
//     if (*a == '\0' && *b == '\0')
//       return &track[i];
//   }
//   return nullptr;
// }

// template <typename GoalFn>
// inline const marklin::TrackNode* bfsAndSetSwitches(const marklin::TrackNode* startNode, GoalFn goalFn,
//                                                    marklin::TrainTrackState& ttState, int dispatcherTid, int uiTid) {
//   bool visited[marklin::NUM_TRACK]{};
//   const marklin::TrackEdge* pred[marklin::NUM_TRACK]{};
//   RingBuffer<const marklin::TrackNode*, 256> bfsQueue;

//   visited[startNode->id] = true;
//   bfsQueue.push(startNode);

//   const marklin::TrackNode* targetNode = nullptr;

//   while (!bfsQueue.empty()) {
//     const marklin::TrackNode* node = bfsQueue.pop();
//     if (goalFn(node)) {
//       targetNode = node;
//       break;
//     }

//     auto tryEdge = [&](const marklin::TrackEdge& edge) {
//       const marklin::TrackNode* dest = edge.dest;
//       if (dest != nullptr && dest->id < marklin::NUM_TRACK && !visited[dest->id]) {
//         visited[dest->id] = true;
//         pred[dest->id] = &edge;
//         bfsQueue.push(dest);
//       }
//     };

//     switch (node->type) {
//     case marklin::TrackNode::Type::Branch:
//       tryEdge(node->edge[marklin::TrackDirection::Straight]);
//       tryEdge(node->edge[marklin::TrackDirection::Curved]);
//       break;
//     case marklin::TrackNode::Type::Sensor:
//     case marklin::TrackNode::Type::Merge:
//     case marklin::TrackNode::Type::Enter:
//       tryEdge(node->edge[marklin::TrackDirection::Ahead]);
//       break;
//     default:
//       break;
//     }
//   }

//   if (targetNode == nullptr) {
//     return nullptr;
//   }

//   const marklin::TrackNode* cur = targetNode;
//   while (cur != startNode && pred[cur->id] != nullptr) {
//     const marklin::TrackEdge* edge = pred[cur->id];
//     const marklin::TrackNode* src = edge->src;
//     if (src->type == marklin::TrackNode::Type::Branch) {
//       marklin::SwitchState sw;
//       if (edge == &src->edge[marklin::TrackDirection::Straight]) {
//         sw = marklin::SwitchState::Straight;
//       } else {
//         sw = marklin::SwitchState::Curved;
//       }
//       uint8_t id = static_cast<uint8_t>(src->id);
//       ttState.setSwitchState(id, sw);
//       sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(id, sw));
//       sendSwitchToUI(uiTid, id, sw);
//     }
//     cur = src;
//   }

//   return targetNode;
// }
} // namespace marklin
