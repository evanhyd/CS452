#include "name_server.h"
#include "fmt.h"
#include "kit_algorithm.h" // strncpy, strncmp
#include "syscalls.h"
#include "uart.h"
#include <cstddef>

namespace {

const int MAX_NAME_LENGTH = 64;

struct Message {
  enum class Type { REGISTER_AS, WHO_IS } type;
  char name[MAX_NAME_LENGTH];
};

struct Entry {
  int tid;
  char name[MAX_NAME_LENGTH];
};

int send(Message::Type type, const char* name) {
  Message msg;
  msg.type = type;
  strncpy(msg.name, name, MAX_NAME_LENGTH);
  int response;
  if (::Send(NAME_SERVER_TID, reinterpret_cast<const char*>(&msg), sizeof(msg), reinterpret_cast<char*>(&response),
             sizeof(response)) < 0) {
    return -1;
  }
  return response;
}

} // namespace

void nameServerTask() {
  static constexpr size_t MAX_ENTRIES = 1024;
  Entry nameTable[MAX_ENTRIES];
  size_t nameCount = 0;
  char buffer[64];

  while (true) {
    Message msg;
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    int response;
    if (msg.type == Message::Type::REGISTER_AS) {
      response = 0;
      bool found = false;
      for (size_t i = 0; i < nameCount; ++i) {
        if (strncmp(nameTable[i].name, msg.name, MAX_NAME_LENGTH) == 0) {
          nameTable[i].tid = senderTid;
          found = true;
          break;
        }
      }
      if (!found && nameCount < MAX_ENTRIES) {
        nameTable[nameCount].tid = senderTid;
        strncpy(nameTable[nameCount].name, msg.name, MAX_NAME_LENGTH);
        ++nameCount;
      } else {
        response = -2;
      }
    } else if (msg.type == Message::Type::WHO_IS) {
      response = -2;
      for (size_t i = 0; i < nameCount; ++i) {
        if (strncmp(nameTable[i].name, msg.name, MAX_NAME_LENGTH) == 0) {
          response = nameTable[i].tid;
          break;
        }
      }
    }
    if (int res = ::Reply(senderTid, reinterpret_cast<const char*>(&response), sizeof(response)); res < 0) {
      kit::formatString(buffer, "NameServer: Reply to %d failed with code %d\r\n", senderTid, res);
      Uart::syncPrint(Uart::CONSOLE, buffer);
    }
  }
}

extern "C" int RegisterAs(const char* name) { return send(Message::Type::REGISTER_AS, name); }

extern "C" int WhoIs(const char* name) { return send(Message::Type::WHO_IS, name); }

void testTask() {
  char buffer[64];
  const char* testName = "TestTask";
  Uart::syncPrint(Uart::CONSOLE, "Trying RegisterAs\r\n");
  int regResult = RegisterAs(testName);
  if (regResult < 0) {
    kit::formatString(buffer, "RegisterAs returned %d\r\n", regResult);
    Uart::syncPrint(Uart::CONSOLE, buffer);
    return;
  }
  Uart::syncPrint(Uart::CONSOLE, "Trying WhoIs\r\n");
  int whoIsResult = WhoIs(testName);
  if (whoIsResult != ::MyTid()) {
    kit::formatString(buffer, "WhoIs returned %d\r\n", whoIsResult);
    Uart::syncPrint(Uart::CONSOLE, buffer);
    return;
  }

  Uart::syncPrint(Uart::CONSOLE, "TestTask passed\r\n");
}
