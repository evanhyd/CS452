#pragma once

namespace clock_server {
int Time(int tid);
int Delay(int tid, int ticks);
int DelayUntil(int tid, int ticks);
void clockServerTask();
} // namespace clock_server
