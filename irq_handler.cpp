#include "irq_handler.h"
#include "debug.h"
#include "fmt.h"
#include "gic.h"
#include "timer.h"

using namespace timer::literals;

namespace irq_handler {
void interruptEntry() {
  auto interruptId = gic::gicc_manager.readAndActivateInterruptId();
  char buf[64];
  kit::formatString(buf, "Interrupt ID: %u\n", static_cast<uint32_t>(interruptId));
  logDebug(buf);
  timer::system_timer.clearChannel1();
  timer::system_timer.setChannel1After(timer::TICK_DURATION);
  gic::gicc_manager.deactivateInterrupt(interruptId);
}
} // namespace irq_handler
