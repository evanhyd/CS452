#include "mcp2515.h"
#include "spi.h"
#include <cstdint>

namespace mcp2515 {

namespace {

// configuration registers
constexpr uint8_t CNF3 = 0x28;
constexpr uint8_t CNF2 = 0x29;
constexpr uint8_t CNF1 = 0x2A;

// MCP2515 configuration for 16 MHz clock and 250 kbit/s bitrate
// Chapter 5.0 Bit Timing in MCP2515 datasheet
// and/or https://kvaser.com/support/calculators/bit-timing-calculator/.
constexpr uint8_t MCP_16MHz_250kbPS_CFG1 = 0x41;
constexpr uint8_t MCP_16MHz_250kbPS_CFG2 = 0xF1;
constexpr uint8_t MCP_16MHz_250kbPS_CFG3 = 0x85;

// MCP2515 normal operation mode
constexpr uint8_t OPMODE_NORMAL = 0x00;

// MCP2515 instruction set
enum Instruction : uint8_t {
  Write = 0x02,
  Read = 0x03,
  BitModify = 0x05,
  ReadStatus = 0xA0,
  LoadTx0 = 0b0100'0000,
  RtsTx0 = 0b1000'0001,
  ReadRx0 = 0b1001'0000,
  ReadRx1 = 0b1001'0100,
};

// MCP2515 status mask
constexpr uint8_t STATUS_RX0 = 1 << 0;
constexpr uint8_t STATUS_RX1 = 1 << 1;
constexpr uint8_t STATUS_TX0 = 1 << 2;

// MCP2515 buffer registers
constexpr uint8_t RXBnCTRL0 = 0x60;
constexpr uint8_t RXBnCTRL1 = 0x70;

// control and status registers
constexpr uint8_t CANSTAT = 0x0E;
constexpr uint8_t CANCTRL = 0x0F;
// OPMOD mask for CANSTAT register
constexpr uint8_t CANSTAT_OPMOD = 0xE0;
// REQOP mask for CANCTRL register
constexpr uint8_t CANCTRL_REQOP = 0xE0;

// flags register
constexpr uint8_t CANINTE = 0x2B;
constexpr uint8_t CANINTF = 0x2C;

/** Read n consecutive registers starting from the specified one. */
void read_regs(uint8_t reg, uint8_t values[], const uint8_t n) {
  spi::begin_transaction();
  spi::transfer_one(Instruction::Read);
  spi::transfer_one(reg);
  for (uint8_t i = 0; i < n; i++) {
    // during transaction, address pointer is automatically incremented after each byte transfer.
    values[i] = spi::transfer_one(0x00);
  }
  spi::end_transaction();
}

enum class RxBuffer : uint8_t {
  Rx0 = 0,
  Rx1 = 1,
};

void read_rx(RxBuffer buffer, uint8_t values[], const uint8_t n) {
  spi::begin_transaction();
  spi::transfer_one(buffer == RxBuffer::Rx0 ? Instruction::ReadRx0 : Instruction::ReadRx1);
  for (uint8_t i = 0; i < n; i++) {
    values[i] = spi::transfer_one(0x00);
  }
  spi::end_transaction();
}

/** Read the value of a single register. */
uint8_t read_reg(uint8_t reg) {
  uint8_t ret = 0;
  read_regs(reg, &ret, 1);
  return ret;
}

/** Write values to n consecutive registers starting from the specified one. */
void write_regs(uint8_t reg, const uint8_t values[], const uint8_t n) {
  spi::begin_transaction();
  spi::transfer_one(Instruction::Write);
  spi::transfer_one(reg);
  for (uint8_t i = 0; i < n; i++) {
    spi::transfer_one(values[i]);
  }
  spi::end_transaction();
}

void write_tx0(const uint8_t values[], const uint8_t n) {
  spi::begin_transaction();
  spi::transfer_one(Instruction::LoadTx0);
  for (uint8_t i = 0; i < n; i++) {
    spi::transfer_one(values[i]);
  }
  spi::end_transaction();
}

/** Write a value to a single register. */
void write_reg(uint8_t reg, const uint8_t value) { write_regs(reg, &value, 1); }

/** Modify individual bits of a register according to mask. */
void modify_reg(uint8_t reg, const uint8_t mask, const uint8_t data) {
  spi::begin_transaction();
  spi::transfer_one(Instruction::BitModify);
  spi::transfer_one(reg);
  spi::transfer_one(mask);
  spi::transfer_one(data);
  spi::end_transaction();
}

/** Read the status of the MCP2515, including RX and TX buffers. */
uint8_t read_status() {
  spi::begin_transaction();
  spi::transfer_one(Instruction::ReadStatus);
  uint8_t ret = spi::transfer_one(0x00);
  spi::end_transaction();
  return ret;
}

void rts_tx0() {
  spi::begin_transaction();
  spi::transfer_one(Instruction::RtsTx0);
  spi::end_transaction();
}

// precondition: tx buffer is free
void transmit_message(const MMessage& msg) {
  uint8_t tx[13];
  tx[0] = (msg.uid >> 21) & 0xFF;
  tx[1] = (msg.uid >> 18 & 0x07) << 5 | 0x08 | (msg.uid >> 16 & 0x03);
  tx[2] = (msg.uid >> 8) & 0xFF;
  tx[3] = msg.uid & 0xFF;
  tx[4] = msg.dlc & 0x0F;
  for (int i = 0; i < 8; i++) {
    tx[5 + i] = i < msg.dlc ? msg.data[i] : 0;
  }
  write_tx0(tx, 5 + msg.dlc);
  rts_tx0();
}

// precondition: specified buffer has data available
void read_message(RxBuffer buffer, MMessage& msg) {
  uint8_t rx[13];
  read_rx(buffer, rx, 13);
  uint32_t sidh = rx[0];
  uint32_t sidl = rx[1];
  uint32_t eid8 = rx[2];
  uint32_t eid0 = rx[3];
  msg.uid = sidh << 21 | (sidl >> 5 & 0x07) << 18 | (sidl & 0x03) << 16 | eid8 << 8 | eid0;
  msg.dlc = rx[4] & 0x0F;
  for (int i = 0; i < 8; i++) {
    msg.data[i] = rx[5 + i];
  }
}

void send_message(const MMessage& msg) {
  // All this logic should be moved to user task
  // if (tx_buffer.empty() && last_command_acked() && !(read_status() & STATUS_TX0)) {
  //   transmit_message(msg);
  //   push_history(msg);
  // } else if (!tx_buffer.full()) {
  //   tx_buffer.push(msg);
  // }
  transmit_message(msg);
}

} // namespace

void init() {
  // No need to reset MCP2515 here as a hardware reset is done during boot.
  // MCP2515 automatically enters config mode after hardware reset.

  // Set the bitrate configuration registers
  write_reg(CNF1, MCP_16MHz_250kbPS_CFG1);
  write_reg(CNF2, MCP_16MHz_250kbPS_CFG2);
  write_reg(CNF3, MCP_16MHz_250kbPS_CFG3);

  // do not filter messages. Allow rollover of RXB0 to RXB1.
  write_reg(RXBnCTRL0, 0x64);
  write_reg(RXBnCTRL1, 0x60);

  // start MCP2515 by setting operation mode to normal
  modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);
  while ((read_reg(CANSTAT) & CANSTAT_OPMOD) != OPMODE_NORMAL)
    ; // wait until mode is set
}

void setInterruptEnabled(CanInterruptType interruptType, bool isEnabled) {
  modify_reg(CANINTE, static_cast<uint8_t>(interruptType), (isEnabled ? 0xff : 0));
}

uint8_t getInterruptStatus() { return read_status(); }

void clearActiveInterrupt(CanInterruptType interruptType) {
  modify_reg(CANINTF, static_cast<uint8_t>(interruptType), 0);
}

} // namespace mcp2515
