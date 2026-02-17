#pragma once
#include "marklin_train_track.h"
#include <cstddef>
#include <cstdint>

namespace marklin {

// Commands
enum class Command : uint8_t {
  SystemCommands = 0x00,
  LokDiscovery = 0x01,
  MfxBind = 0x02,
  MfxVerify = 0x03,
  TrainSpeed = 0x04,
  TrainDirection = 0x05,
  TrainFunction = 0x06,
  ReadConfig = 0x07,
  WriteConfig = 0x08,
  AccessoriesSwitching = 0x0B,
  AccessoriesConfiguration = 0x0C,
  S88Polling = 0x10,
  FeedbackEvent = 0x11,
};

// SystemCommands Sub-Commands
enum class SystemSub : uint8_t {
  Stop = 0x00,                      // DLC 5
  Go = 0x01,                        // DLC 5
  Halt = 0x02,                      // DLC 5
  EmergencyStop = 0x03,             // DLC 5
  CycleStop = 0x04,                 // DLC 5
  DataProtocol = 0x05,              // DLC 6
  SwitchingTimeAccessory = 0x06,    // DLC 5
  MfxFastRead = 0x07,               // DLC 6
  UnlockTrackProtocol = 0x08,       // DLC 6
  MfxRegistrationCounterSet = 0x09, // DLC 7
  Overload = 0x0A,                  // DLC 6
  Status = 0x0B,                    // DLC 6, 8
  Identifier = 0x0C,                // DLC 5, 7
  MfxSeek = 0x30,                   // DLC 6, 7, 8
  Reset = 0x80,                     // DLC 6
};

struct SensorEvent {
  char bank;
  uint8_t number;
  bool old_occupied;
  bool new_occupied;
};

struct MMessage {
  uint8_t priority; // 4 bits
  Command command;  // 8 bits
  uint8_t response; // 1 bit
  uint16_t hash;    // 16 bits
  uint8_t dlc;      // 4 bits
  uint8_t data[8];  // 8 * 8 bits (8 byte data)

  static MMessage base(uint8_t priority, Command command, uint8_t dlc, uint8_t data[]);
  static MMessage systemStopAll();
  static MMessage systemGoAll();
  static MMessage systemHaltAll();
  static MMessage systemEmergencyStop(uint8_t trainNumber);
  static MMessage setTrainSpeed(uint8_t trainNumber, uint16_t speed);
  static MMessage getTrainSpeed(uint8_t trainNumber);
  static MMessage getTrainDirection(uint8_t trainNumber);
  static MMessage setTrainDirection(uint8_t trainNumber, TrainDirection direction);
  static MMessage getTrainFunctionState(uint8_t trainNumber, TrainFunction function);
  static MMessage setTrainFunctionState(uint8_t trainNumber, TrainFunction function, uint8_t value);
  static MMessage setSwitchState(uint8_t switchNumber, SwitchState state, bool isSolenoidActive);
};
} // namespace marklin
