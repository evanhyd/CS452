#pragma once

#include <cstdint>

namespace k4 {

void keypressTask();

void eventListenerTask();

void clockTask();

struct ReverseArgs {
  // avoid an extra WhoIs call... does it really matter? no
  int clockServerTid;
  uint8_t trainNo;
  uint8_t oldSpeed;
};

void reverseWorkerTask();

} // namespace k4
