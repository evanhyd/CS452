#pragma once

#include "mcp2515.h"
namespace can_server {

inline constexpr const char* CAN_SERVER_NAME = "can_server";

void canServerTask();

} // namespace can_server

extern "C" {
int ReadCAN(int tid, mcp2515::MMessage* msg);
int TransmitCAN(int tid, const mcp2515::MMessage* msg);
}
