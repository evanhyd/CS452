#pragma once

#include <cstdint>

namespace spi {

void init();
void begin_transaction();
void end_transaction();
uint8_t transfer_one(uint8_t tx_byte);

} // namespace spi
