#pragma once

#include <cstdint>

namespace gpio {

void init();

// Get event detect status for GPIO pin.
uint32_t get_event_detect_status(uint32_t pin);

// Clear event detect status for GPIO pin.
void clr_event_detect_status(uint32_t pin);

} // namespace gpio
