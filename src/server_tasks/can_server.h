#pragma once
#include "marklin/marklin_message.h"
#include "kernel/devices/mcp2515.h"

namespace marklin {
struct MMessage;
}

namespace can_server {

inline constexpr const char* CAN_SERVER_NAME = "can_server";

void canServerTask();

} // namespace can_server

int ReceiveCAN(int tid, marklin::MMessage& msg);
int TransmitCAN(int tid, const marklin::MMessage& msg);
