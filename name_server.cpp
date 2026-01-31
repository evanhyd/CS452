#include "name_server.h"
#include "debug.h"
#include "fmt.h"
#include "kit_algorithm.h" // strncpy, strncmp
#include "static_stack.h"
#include "syscalls.h"
#include "task_manager.h"
#include "task_queue.h"
#include "uart.h"
#include <cstddef>

namespace {

constexpr size_t MAX_NAME_LENGTH = 64;
constinit Tid nameServerTid = Tid::invalid();

struct Message {
  enum class Type : uint8_t { REGISTER_AS, WHO_IS } type;
  char name[MAX_NAME_LENGTH];
};

struct NameEntry {
  int tid;
  char name[MAX_NAME_LENGTH];
};

// Send a query to the name server.
// Return the reseponse code that depends on the query type.
int send(Message::Type type, const char* name) {
  Message msg;
  msg.type = type;
  strncpy(msg.name, name, MAX_NAME_LENGTH);
  int response;
  if (::Send(nameServerTid.raw(), reinterpret_cast<const char*>(&msg), sizeof(msg), reinterpret_cast<char*>(&response),
             sizeof(response)) < 0) {
    return -1;
  }
  return response;
}

void nameServerTask() {
  static constexpr size_t MAX_ENTRIES = 128;
  StaticStack<NameEntry, MAX_ENTRIES> nameTable;
  char buffer[64];

  // Register the server name.
  const auto registerHandler = [&](const Message& msg, int senderTid) {
    for (NameEntry& entry : nameTable) {
      if (strncmp(entry.name, msg.name, MAX_NAME_LENGTH) == 0) {
        entry.tid = senderTid;
        return 0;
      }
    }

    if (!nameTable.full()) {
      nameTable.push(NameEntry{});
      nameTable.top().tid = senderTid;
      strncpy(nameTable.top().name, msg.name, MAX_NAME_LENGTH);
      return 0;
    }

    // Run out of name entry, *should not happen.
    return -1;
  };

  // Lookup the server name.
  const auto whoIsHandler = [&](const Message& msg, int) {
    for (const NameEntry& entry : nameTable) {
      if (strncmp(entry.name, msg.name, MAX_NAME_LENGTH) == 0) {
        return entry.tid;
      }
    }
    return -2;
  };

  while (true) {
    Message msg;
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    int response;

    switch (msg.type) {
    case Message::Type::REGISTER_AS:
      response = registerHandler(msg, senderTid);
      break;
    case Message::Type::WHO_IS:
      response = whoIsHandler(msg, senderTid);
      break;
    }

    if (int res = ::Reply(senderTid, reinterpret_cast<const char*>(&response), sizeof(response)); res < 0) {
      kit::formatString(buffer, "NameServer: Reply to %d failed with code %d\r\n", senderTid, res);
      Uart::syncPrint(Uart::CONSOLE, buffer);
    }
  }
}

} // namespace

extern "C" {
// Registers the task id of the caller under the given name.
// Return Value
// 0	success.
// -1	unable to reach name server.
// -2   if the name entry is full.
int RegisterAs(const char* name) {
  if (nameServerTid == Tid::invalid()) {
    return -1;
  }
  return send(Message::Type::REGISTER_AS, name);
}

// Asks the name server for the task id of the task that is registered under the given name.
// Return Value
// tid	task id of the registered task.
// -1	unable to reach name server.
// -2   if no such name is registered.
int WhoIs(const char* name) {
  if (nameServerTid == Tid::invalid()) {
    return -1;
  }
  return send(Message::Type::WHO_IS, name);
}
}

namespace name_server {

// Create a name server and register its task id internally.
// Return the task id.
int createNameServerTask(int priority) {
  nameServerTid = Tid::fromRaw(::Create(priority, nameServerTask));
  return nameServerTid.raw();
}

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

  // Register twice, should override without error.
  regResult = RegisterAs(testName);
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
} // namespace name_server
