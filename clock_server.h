#pragma once

namespace clock_server {

inline constexpr const char* CLOCK_SERVER_NAME = "clock_server";

extern "C" {
int Time(int tid);
int Delay(int tid, int ticks);
int DelayUntil(int tid, int ticks);
}

void clockServerTask();

} // namespace clock_server
