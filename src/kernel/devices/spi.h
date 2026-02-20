#pragma once
#include <cstdint>

/**
Serial Peripheral Interface
    Used for communication with MCP2515 / CAN bus interface.
    BCM Chap 9 (not Section 2.3): SPI0.
    Device registers at offset 0x204000.
    Not just a single standard; very configurable and flexible.
    Synchronous bidirectional transmission of bytes, 1-3 in, same out.
    "transaction" of up to N bytes.
    Actual sending/receiving happens in parallel to processing.
    Semantics determined by usage; FIFOs not used in our code.
 */
namespace spi {
void init();
void beginTransaction();
void endTransaction();
uint8_t transferOne(uint8_t tx_byte);
} // namespace spi
