#include "k4_tasks.h"
#include "can_server.h"
#include "clock_server.h"
#include "console.h"
#include "ctfmt.h"
#include "debug.h"
#include "fmt.h"
#include "io_server.h"
#include "kit_algorithm.h"
#include "marklin_event.h"
#include "marklin_message.h"
#include "marklin_train_track.h"
#include "name_server.h"
#include "static_stack.h"
#include "syscalls.h"

namespace {
void eventListener() {
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);

  for (;;) {
    marklin::MMessage message;
    ::ReceiveCAN(canServerTid, message);

    // Must be a respoonse.
    if (!message.response) {
      continue;
    }

    // Broadcast the event.
    char buffer[256] = {};
    switch (message.command) {
    case marklin::Command::AccessoriesSwitching: {
      switch (message.dlc) {
      case 6: {
        marklin::SwitchStateEvent event(message);
        event.toString(buffer);
        break;
      }
      }
      break;
    }
    case marklin::Command::TrainSpeed: {
      switch (message.dlc) {
      case 6: {
        marklin::TrainSpeedEvent event(message);
        event.toString(buffer);
        break;
      }
      }
      break;
    }
    case marklin::Command::TrainDirection: {
      switch (message.dlc) {
      case 5: {
        marklin::TrainDirectionEvent event(message);
        event.toString(buffer);
        break;
      }
      }
      break;
    }
    case marklin::Command::TrainFunction: {
      switch (message.dlc) {
      case 6: {
        marklin::TrainFunctionEvent event(message);
        event.toString(buffer);
        break;
      }
      }
      break;
    }
    case marklin::Command::FeedbackEvent: {
      switch (message.dlc) {
      case 8: {
        marklin::SensorTriggeredEvent event(message);
        event.toString(buffer);
        break;
      }
      }
      break;
    }
    default: {
      break;
    }
    }

    // Print the event message.
    size_t bufferLen = ::strlen(buffer);
    for (size_t i = 0; i < bufferLen; ++i) {
      ::Putc(ioServerTid, buffer[i]);
    }
    if (bufferLen != 0) {
      ::Putc(ioServerTid, '\r');
      ::Putc(ioServerTid, '\n');
    }
  }
}

void terminalInput() {
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  int canServerTid = ::WhoIs(can_server::CAN_SERVER_NAME);

  for (;;) {

    StaticStack<char, 256> textInput{};
    for (;;) {
      char c = char(::Getc(ioServerTid));
      if (c == '\r' || c == '\n') {
        break;
      }
      if (c == '\b' && !textInput.empty()) {
        textInput.pop();
      }
      if (kit::isPrintable(c)) {
        textInput.push(c);
        ::Putc(ioServerTid, c);
      }
    }

    const char* tokenBegin = NULL;
    const char* tokenEnd = textInput.begin();
    if (!kit::extractStr(&tokenBegin, &tokenEnd, ' ')) {
      return;
    }

    size_t tokenSize = size_t(tokenEnd - tokenBegin);
    if (!strncmp(tokenBegin, "tr", tokenSize)) {
      logDebug("set train speed function");

      // Extract train number.
      uint8_t trainNumber = 0;
      if (!kit::extractU8(&tokenBegin, &tokenEnd, ' ', &trainNumber)) {
        return;
      }

      // Extract speed.
      uint16_t speed = 0;
      if (!kit::extractU16(&tokenBegin, &tokenEnd, ' ', &speed)) {
        return;
      }

      marklin::MMessage message = marklin::MMessage::setTrainSpeed(trainNumber, speed);
      ::TransmitCAN(canServerTid, message);

    } else if (!strncmp(tokenBegin, "rv", tokenSize)) {
      // // Extract train number.
      // uint8_t trainNumber = 0;
      // if (!kit::extractU8(&tokenBegin, &tokenEnd, ' ', &trainNumber)) {
      //   return;
      // }

      // uint16_t oldTrainSpeed = board.getTrainSpeed(trainNumber);

      // // Set the train speed to 0.
      // marklin::MMessage message = marklin::MMessage::setTrainSpeed(trainNumber, 0);
      // taskQueue.sendCommand(SystemTimer::counterValue(), message);

      // // Send reverse direction after 5 seconds.
      // message = marklin::MMessage::setTrainDirection(trainNumber, TOGGLE);
      // taskQueue.sendCommand(SystemTimer::counterValue() + SystemTimer::millisecondsToTicks(5000), message);

      // // Speed up again.
      // message = marklin::MMessage::setTrainSpeed(trainNumber, oldTrainSpeed);
      // taskQueue.sendCommand(SystemTimer::counterValue(), message);

    } else if (!strncmp(tokenBegin, "sw", tokenSize)) {
      // // Extract switch number.
      // uint8_t switchNumber = 0;
      // if (!kit::extractU8(&tokenBegin, &tokenEnd, ' ', &switchNumber)) {
      //   return;
      // }

      // // Extract switch direction.
      // if (!kit::extractStr(&tokenBegin, &tokenEnd, ' ')) {
      //   return;
      // }
      // uint8_t switchState = ((tokenBegin[0] == 'C' || tokenBegin[0] == 'c') ? 0 : 1);

      // // Activate solenoid to change the direction.
      // marklin::MMessage message =
      //     marklin::MMessage::setSwitchState(switchNumber, marklin::SwitchState(switchState), true);
      // taskQueue.sendCommand(SystemTimer::counterValue(), message);

      // // Wait a bit, then turn off the solenoid.
      // constexpr uint32_t WAIT_TIME_MS = 100;
      // message = marklin::MMessage::setSwitchState(switchNumber, marklin::SwitchState(switchState), false);
      // taskQueue.sendCommand(SystemTimer::counterValue() + SystemTimer::millisecondsToTicks(WAIT_TIME_MS), message);

    } else if (!strncmp(tokenBegin, "fn", tokenSize)) {
      // Extract train number.
      uint8_t trainNumber = 0;
      if (!kit::extractU8(&tokenBegin, &tokenEnd, ' ', &trainNumber)) {
        return;
      }

      // Extract train function.
      uint8_t function = 0;
      if (!kit::extractU8(&tokenBegin, &tokenEnd, ' ', &function)) {
        return;
      }

      // Extract function value.
      uint8_t value = 0;
      if (!kit::extractU8(&tokenBegin, &tokenEnd, ' ', &value)) {
        return;
      }

      marklin::MMessage message =
          marklin::MMessage::setTrainFunctionState(trainNumber, marklin::TrainFunction(function), value);
      ::TransmitCAN(canServerTid, message);
    }
  }
}

constexpr const char* UI_SERVER_NAME = "ui_server";

enum class UiMessageType : int {
  Timer,
};

struct TimerUpdate {
  unsigned deciseconds;
};

struct UiMessage {
  UiMessageType type;
  union {
    TimerUpdate timerUpdate;
  };
};

void uiServerTask() {
  if (::RegisterAs(UI_SERVER_NAME) < 0) {
    logError("failed to register itself to name server");
  }
  int ioServerTid = ::WhoIs(io_server::IO_SERVER_NAME);
  if (ioServerTid < 0) {
    logError("failed to find IO server");
  }
  Console console{ioServerTid};
  console.clearScreen();
  console.hideCursor();
  UiMessage msg;
  for (;;) {
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);
    switch (msg.type) {
    case UiMessageType::Timer: {
      console.moveCursor(1, 1);
      unsigned mins = msg.timerUpdate.deciseconds / 600;
      unsigned secs = msg.timerUpdate.deciseconds / 10 % 60;
      unsigned tenths = msg.timerUpdate.deciseconds % 10;
      console.printf("%02u:%02u.%u", mins, secs, tenths);
      break;
    }
    default:
      logError("Received unknown message type");
    }
  }
}

void clockTask() {
  int clockServerTid = ::WhoIs(clock_server::CLOCK_SERVER_NAME);
  if (clockServerTid < 0) {
    logError("failed to find clock server");
  }
  int uiServerTid = ::WhoIs(UI_SERVER_NAME);
  if (uiServerTid < 0) {
    logError("failed to find UI server");
  }
  for (;;) {
    int ticks = ::Delay(clockServerTid, 10); // 10 ticks = 100 ms
    if (ticks < 0) {
      logError("Delay failed");
    }
    unsigned deciseconds = static_cast<unsigned>(ticks) / 10;
    UiMessage msg{.type = UiMessageType::Timer, .timerUpdate{deciseconds}};
    char devnull;
    if (::Send(uiServerTid, reinterpret_cast<const char*>(&msg), sizeof(msg), &devnull, 0) < 0) {
      logError("failed to send timer update to UI server");
    }
  }
}

} // namespace

void k4::FirstUserTask() {
  if (name_server::createNameServerTask(1) < 0) {
    logError("failed to create name server task");
  }
  int ioServerTid = ::Create(1, io_server::ioServerTask);
  if (ioServerTid < 0) {
    logError("failed to create ioServerTask");
  }
  int canServerTid = ::Create(1, can_server::canServerTask);
  if (canServerTid < 0) {
    logError("failed to create canServerTask");
  }
  int clockServerTid = ::Create(1, clock_server::clockServerTask);
  if (clockServerTid < 0) {
    logError("failed to create clockServerTask");
  }

  // ::Create(2, eventListener);
  // ::Create(2, terminalInput);
  ::Create(2, uiServerTask);
  ::Create(8, clockTask);
}
