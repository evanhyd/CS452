#include "mcp2515.h"
#include "marklin_message.h"
#include "spi.h"
#include <cstdint>

namespace {
/**
 * Transmit Buffer (3)
 * MCP2515-Family-Data-Sheet-DS20001801K page 15
 */
// Registers
constexpr uint8_t TXBnCTRL = 0x30; // control register
constexpr uint8_t TXBnSIDH = 0x31; // standard identifier reigster high
constexpr uint8_t TXBnSIDL = 0x32; // standard identifier reigster low
constexpr uint8_t TXBnEIDH = 0x33; // extended identifier reigster high
constexpr uint8_t TXBnEIDL = 0x34; // extended identifier reigster low
constexpr uint8_t TXBnDLC = 0x35;  // data length code register
constexpr uint8_t TXBnDm = 0x36;   // data byte register (8 bytes buffer)

// Masks
constexpr uint8_t TXP = 0b11;       // control register: priority (00 lowest)
constexpr uint8_t TXREQ = 1 << 3;   // control register: message transmit request
constexpr uint8_t TXERR = 1 << 4;   // control register: message transmit error detected
constexpr uint8_t MLOA = 1 << 5;    // control register: message lost arbitration
constexpr uint8_t ABTF = 1 << 6;    // control register: message aborted flag
constexpr uint8_t SID_10_3 = 0xff;  // standard identifier register high: identifier
constexpr uint8_t SID_2_0 = 7 << 5; // standard identifier register low: identifier
constexpr uint8_t EXIDE = 1 << 3;   // standard identifier register low: extended identifier enable
constexpr uint8_t EID_17_16 = 0b11; // standard identifier register low: extended identifier
constexpr uint8_t EID_15_8 = 0xff;  // extended identifier register high: extended identifier
constexpr uint8_t EID_7_0 = 0xff;   // extended identifier register low: extended identifier
constexpr uint8_t RTR =
    1 << 6; // data length code register: remote transmission request (1. remote transmit request | 0. data frame)
constexpr uint8_t DLC = 0xf; // data length code register: data length code

/**
 * Receive Buffer (2)
 * MCP2515-Family-Data-Sheet-DS20001801K page 23
 */
// Registers
constexpr uint8_t RXBnCTRL0 = 0x60; // control register 0
constexpr uint8_t RXBnSIDH0 = 0x61; // standard identifer register high 0
constexpr uint8_t RXBnSIDL0 = 0x62; // standard identifer register low 0
constexpr uint8_t RXBnEIDH0 = 0x63; // extended identifer register high 0
constexpr uint8_t RXBnEIDL0 = 0x64; // extended identifer register low 0
constexpr uint8_t RXBnDLC0 = 0x65;  // data length code register 0
constexpr uint8_t RXBnDm0 = 0x66;   // data byte register 0 (8 bytes)
constexpr uint8_t RXBnCTRL1 = 0x70; // control register 1
constexpr uint8_t RXBnSIDH1 = 0x71; // standard identifer register high 1
constexpr uint8_t RXBnSIDL1 = 0x72; // standard identifer register low 1
constexpr uint8_t RXBnEIDH1 = 0x73; // extended identifer register high 1
constexpr uint8_t RXBnEIDL1 = 0x74; // extended identifer register low 1
constexpr uint8_t RXBnDLC1 = 0x75;  // data length code register 1
constexpr uint8_t RXBnDm1 = 0x76;   // data byte register 1 (8 bytes)

/**
 * SPI Instruction Set
 * MCP2515-Family-Data-Sheet-DS20001801K page 67
 */
enum Instruction : uint8_t {
  Reset = 0b1100'0000,   // Resets internal registers to the default state, sets Configuration mode.
  Read = 0x03,           // Reads data from the register beginning at selected address.
  ReadRx0 = 0b1001'0000, // Faster READ command directly to RX0.
                         //  Note: The associated RX flag bit, RXnIF (CANINTF), will be cleared after bringing CS high.
  ReadRx1 = 0b1001'0100,
  Write = 0x02,          // Writes data to the register beginning at the selected address.
  LoadTx0 = 0b0100'0000, // Faster WRITE command directly to TX0.
  RtsTx0 = 0b1000'0001,  // Instructs controller to begin message transmission sequence.
  ReadStatus = 0xA0,     // Reads several status bits for transmit and receive functions.
  BitModify = 0x05,      // Allows the user to set or clear individual bits in a particular register.
};

// Masks
constexpr uint8_t READ_RX_BUFFER_RXB0SIDH = 0b00 << 1; // Receive Buffer 0, Start at RXB0SIDH
constexpr uint8_t READ_RX_BUFFER_RXB0D0 = 0b01 << 1;   // Receive Buffer 0, Start at RXB0D0
constexpr uint8_t READ_RX_BUFFER_RXB1SIDH = 0b10 << 1; // Receive Buffer 1, Start at RXB1SIDH
constexpr uint8_t READ_RX_BUFFER_RXB1D0 = 0b11;        // Receive Buffer 1, Start at RXB1D0
constexpr uint8_t LOAD_TX_BUFFER_TXB0SIDH = 0b000;     // TX Buffer 0, Start at TXB0SIDH
constexpr uint8_t LOAD_TX_BUFFER_TXB0D0 = 0b001;       // TX Buffer 0, Start at TXB0D0
constexpr uint8_t RTS_TXB0 = 0b001;                    // Send TX Buffer 0
constexpr uint8_t RTS_TXB1 = 0b010;                    // Send TX Buffer 1
constexpr uint8_t RTS_TXB2 = 0b100;                    // Send TX Buffer 2
constexpr uint8_t READ_STATUS_RX0IF = 1 << 0;          // Receive buffer 0 contains a message
constexpr uint8_t READ_STATUS_RX1IF = 1 << 1;          // Receive buffer 1 contains a message
constexpr uint8_t READ_STATUS_TX0REQ = 1 << 2;         // Transmit buffer 0 is pending transmission
constexpr uint8_t READ_STATUS_TX0IF = 1 << 3;          // Transmit buffer 0 is empty (interrupt flag)
constexpr uint8_t READ_STATUS_TX1REQ = 1 << 4;         // Transmit buffer 1 is pending transmission
constexpr uint8_t READ_STATUS_TX1IF = 1 << 5;          // Transmit buffer 1 is empty (interrupt flag)
constexpr uint8_t READ_STATUS_TX2REQ = 1 << 6;         // Transmit buffer 2 is pending transmission
constexpr uint8_t READ_STATUS_TX2IF = 1 << 7;          // Transmit buffer 2 is empty (interrupt flag)

// Interrupt registers
constexpr uint8_t CANINTE = 0x2B;
constexpr uint8_t CANINTF = 0x2C;

// Read n consecutive registers starting from the specified one.
void readRegs(uint8_t reg, uint8_t values[], uint8_t n) {
  spi::beginTransaction();
  spi::transferOne(Instruction::Read);
  spi::transferOne(reg);
  // During transaction, address pointer is automatically incremented after each byte transfer.
  for (uint8_t i = 0; i < n; i++) {
    values[i] = spi::transferOne(0x00);
  }
  spi::endTransaction();
}

// Reads data from the register beginning at selected address.
uint8_t readReg(uint8_t reg) {
  uint8_t ret = 0;
  readRegs(reg, &ret, 1);
  return ret;
}

// Write values to n consecutive registers starting from the specified one.
void writeRegs(uint8_t reg, const uint8_t values[], uint8_t n) {
  spi::beginTransaction();
  spi::transferOne(Instruction::Write);
  spi::transferOne(reg);
  for (uint8_t i = 0; i < n; i++) {
    spi::transferOne(values[i]);
  }
  spi::endTransaction();
}

// Write a value to a single register.
void writeReg(uint8_t reg, uint8_t value) { writeRegs(reg, &value, 1); }

// Quick polling command that reads several status bits for transmit and receive functions.
uint8_t readStatus() {
  spi::beginTransaction();
  spi::transferOne(Instruction::ReadStatus);
  uint8_t ret = spi::transferOne(0x00);
  spi::endTransaction();
  return ret;
}

// Modify individual bits of a register according to mask.
void modifyReg(uint8_t reg, uint8_t mask, const uint8_t data) {
  spi::beginTransaction();
  spi::transferOne(Instruction::BitModify);
  spi::transferOne(reg);
  spi::transferOne(mask);
  spi::transferOne(data);
  spi::endTransaction();
}

// Serialize MMessage to the buffer.
void serializeMessage(const marklin::MMessage& message, uint8_t buffer[13]) {
  buffer[0] = (message.priority << 4) | (uint8_t(message.command) >> 4);
  buffer[1] =
      ((uint8_t(message.command) << 4) & 0b11100000) | EXIDE | ((uint8_t(message.command) & 1) << 1) | message.response;
  buffer[2] = uint8_t(message.hash >> 8);
  buffer[3] = uint8_t(message.hash & 0xff);
  buffer[4] = message.dlc;
  buffer[5] = message.data[0];
  buffer[6] = message.data[1];
  buffer[7] = message.data[2];
  buffer[8] = message.data[3];
  buffer[9] = message.data[4];
  buffer[10] = message.data[5];
  buffer[11] = message.data[6];
  buffer[12] = message.data[7];
}

// Deserialize the buffer to the message.
void deserializeMessage(marklin::MMessage& message, const uint8_t buffer[13]) {
  message.priority = buffer[0] >> 4;
  message.command = marklin::Command((buffer[0] << 4) | ((buffer[1] >> 4) & 0b00001110) | ((buffer[1] >> 1) & 1));
  message.response = buffer[1] & 1;
  message.hash = (uint16_t)(buffer[2] << 8) | (uint16_t)(buffer[3]);
  message.dlc = buffer[4];
  message.data[0] = buffer[5];
  message.data[1] = buffer[6];
  message.data[2] = buffer[7];
  message.data[3] = buffer[8];
  message.data[4] = buffer[9];
  message.data[5] = buffer[10];
  message.data[6] = buffer[11];
  message.data[7] = buffer[12];
}

} // namespace

namespace mcp2515 {

// No need to reset MCP2515 here as a hardware reset is done during boot.
// MCP2515 automatically enters config mode after hardware reset.
void init() {
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

  // Control and status registers
  constexpr uint8_t CANSTAT = 0x0E;
  constexpr uint8_t CANCTRL = 0x0F;

  // OPMOD mask for CANSTAT register
  constexpr uint8_t CANSTAT_OPMOD = 0xE0;

  // REQOP mask for CANCTRL register
  constexpr uint8_t CANCTRL_REQOP = 0xE0;

  // MCP2515 normal operation mode
  constexpr uint8_t OPMODE_NORMAL = 0x00;

  // Set the bitrate configuration registers
  writeReg(CNF1, MCP_16MHz_250kbPS_CFG1);
  writeReg(CNF2, MCP_16MHz_250kbPS_CFG2);
  writeReg(CNF3, MCP_16MHz_250kbPS_CFG3);

  // Do not filter messages. Allow rollover of RXB0 to RXB1.
  writeReg(RXBnCTRL0, 0x64);
  writeReg(RXBnCTRL1, 0x60);

  // Start MCP2515 by setting operation mode to normal.
  modifyReg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);
  while ((readReg(CANSTAT) & CANSTAT_OPMOD) != OPMODE_NORMAL)
    ;
}

void setInterruptEnabled(CanInterruptType interruptType, bool isEnabled) {
  modifyReg(CANINTE, static_cast<uint8_t>(interruptType), (isEnabled ? 0xff : 0));
}

uint8_t getInterruptStatus() { return readStatus(); }

void clearActiveInterrupt(CanInterruptType interruptType) {
  modifyReg(CANINTF, static_cast<uint8_t>(interruptType), 0);
}

void sendMessage(const marklin::MMessage& message) {
  // Prepare data.
  uint8_t data[13] = {0};
  serializeMessage(message, data);

  // Write to the transmit register.
  spi::beginTransaction();
  spi::transferOne(Instruction::LoadTx0);
  for (uint8_t i = 0; i < sizeof(data); ++i) {
    spi::transferOne(data[i]);
  }
  spi::endTransaction();

  // Instruct to send.
  spi::beginTransaction();
  spi::transferOne(Instruction::RtsTx0);
  spi::endTransaction();
}

marklin::MMessage receiveMessage(RxBuffer buffer) {
  uint8_t data[13] = {};
  spi::beginTransaction();
  spi::transferOne(buffer == RxBuffer::Rx0 ? Instruction::ReadRx0 : Instruction::ReadRx1);
  for (uint8_t i = 0; i < sizeof(data); ++i) {
    data[i] = spi::transferOne(0);
  }
  spi::endTransaction();

  marklin::MMessage message;
  deserializeMessage(message, data);
  return message;
}

} // namespace mcp2515
