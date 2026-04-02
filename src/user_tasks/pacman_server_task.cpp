#include "pacman_server_task.h"
#include "kernel/syscalls.h"
#include "message.h"

namespace k4 {
void pacmanServerTask() {
  if (::RegisterAs(PACMAN_SERVER_NAME) < 0) {
    logError("failed to register");
  }
}
} // namespace k4