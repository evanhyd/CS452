#include "marklin/marklin_message.h"

namespace {
// Magic Constant from the CS2 Manual and lecture note.
constexpr uint16_t CONTROLLER_HASH = 0xC300;

// LOC-ID Base
enum class TrainProtocol { MM = 0x00, DC = 0xC0 };
constexpr uint8_t TRAIN_BASE = uint8_t(TrainProtocol::MM);
constexpr uint8_t SWITCH_BASE = 0x30;
} // namespace

namespace marklin {
MMessage MMessage::base(uint8_t priority, Command command, uint8_t dlc, uint8_t data[]) {
  MMessage MMessage = {
      .priority = priority, .command = command, .response = 0, .hash = CONTROLLER_HASH, .dlc = dlc, .data = {0}};
  for (size_t i = 0; i < dlc; ++i) {
    MMessage.data[i] = data[i];
  }
  return MMessage;
}

MMessage MMessage::systemStopAll() {
  uint8_t data[] = {0x00, 0x00, 0x00, 0x00, uint8_t(SystemSub::Stop)};
  return MMessage::base(0, Command::SystemCommands, sizeof(data), data);
}

MMessage MMessage::systemGoAll() {
  uint8_t data[] = {0x00, 0x00, 0x00, 0x00, uint8_t(SystemSub::Go)};
  return MMessage::base(0, Command::SystemCommands, sizeof(data), data);
}

MMessage MMessage::systemHaltAll() {
  uint8_t data[] = {0x00, 0x00, 0x00, 0x00, uint8_t(SystemSub::Halt)};
  return MMessage::base(0, Command::SystemCommands, sizeof(data), data);
}

MMessage MMessage::systemEmergencyStop(uint8_t trainNumber) {
  uint8_t data[] = {0x00, 0x00, TRAIN_BASE, trainNumber, uint8_t(SystemSub::EmergencyStop)};
  return MMessage::base(0, Command::SystemCommands, sizeof(data), data);
}

MMessage MMessage::setTrainSpeed(uint8_t trainNumber, uint16_t speed) {
  uint8_t data[] = {0x00, 0x00, TRAIN_BASE, trainNumber, uint8_t(speed >> 8), uint8_t(speed)};
  return MMessage::base(0, Command::TrainSpeed, sizeof(data), data);
}

MMessage MMessage::getTrainSpeed(uint8_t trainNumber) {
  uint8_t data[] = {0x00, 0x00, TRAIN_BASE, trainNumber};
  return MMessage::base(0, Command::TrainSpeed, sizeof(data), data);
}

MMessage MMessage::getTrainDirection(uint8_t trainNumber) {
  uint8_t data[] = {0x00, 0x00, TRAIN_BASE, trainNumber};
  return MMessage::base(0, Command::TrainDirection, sizeof(data), data);
}

MMessage MMessage::setTrainDirection(uint8_t trainNumber, TrainDirection direction) {
  uint8_t data[] = {0x00, 0x00, TRAIN_BASE, trainNumber, (uint8_t)direction};
  return MMessage::base(0, Command::TrainDirection, sizeof(data), data);
}

MMessage MMessage::getTrainFunctionState(uint8_t trainNumber, TrainFunction function) {
  uint8_t data[] = {0x00, 0x00, TRAIN_BASE, trainNumber, (uint8_t)function};
  return MMessage::base(0, Command::TrainFunction, sizeof(data), data);
}

MMessage MMessage::setTrainFunctionState(uint8_t trainNumber, TrainFunction function, uint8_t value) {
  uint8_t data[] = {0x00, 0x00, TRAIN_BASE, trainNumber, (uint8_t)function, value};
  return MMessage::base(0, Command::TrainFunction, sizeof(data), data);
}

MMessage MMessage::setSwitchState(uint8_t switchNumber, SwitchState state, bool isSolenoidActive) {
  --switchNumber; // 0 base index
  uint8_t data[] = {0x00, 0x00, SWITCH_BASE, switchNumber, uint8_t(state), (uint8_t)isSolenoidActive};
  return MMessage::base(0, Command::AccessoriesSwitching, sizeof(data), data);
}
} // namespace marklin
