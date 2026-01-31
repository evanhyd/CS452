#include "k2_tasks.h"
#include "name_server.h"
#include "syscalls.h"
#include "task_queue.h"

namespace k2 {

void rpcServerTask() {}

void rpcClientTask() {}

void FirstUserTask() {
  // Creates the name server.
  name_server::createNameServerTask(Priority::MEDIUM);

  // Creates the Rock/Paper/Scissors server.

  // Creates the Rock/Paper/Scissors clients.
}
} // namespace k2
