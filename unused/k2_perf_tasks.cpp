#include "k2_perf_tasks.h"
#include "fmt.h"
#include "syscalls.h"
#include "task_queue.h"
#include "timer.h"
#include "uart.h"

namespace {

constexpr int REPETITIONS = 100'000;

template <int MessageSize> void sender() {
  char message[(size_t)MessageSize]{};
  for (char& c : message) {
    c = 'A';
  }
  int spawnerTid;
  int receiverTid;
  ::Receive(&spawnerTid, reinterpret_cast<char*>(&receiverTid), sizeof(int));
  ::Reply(spawnerTid, message, 0);

  auto start = timer::system_timer.now();
  for (int i = 0; i < REPETITIONS; ++i) {
    ::Send(receiverTid, message, MessageSize, message, MessageSize);
  }
  auto elapsed = timer::system_timer.since(start);
  ::Send(spawnerTid, reinterpret_cast<const char*>(&elapsed), sizeof(uint32_t), message, 0);
}

template <int MessageSize> void receiver() {
  char message[(size_t)MessageSize]{};
  for (char& c : message) {
    c = 'A';
  }
  int senderTid;
  for (int i = 0; i < REPETITIONS; ++i) {
    ::Receive(&senderTid, message, MessageSize);
    ::Reply(senderTid, message, MessageSize);
  }
}

template <int MessageSize, bool ReceiveFirst> void testPair() {
  int receiverTid = ::Create(ReceiveFirst ? Priority::MEDIUM : Priority::LOW, receiver<MessageSize>);
  int senderTid = ::Create(ReceiveFirst ? Priority::LOW : Priority::MEDIUM, sender<MessageSize>);

  char dummy{};
  ::Send(senderTid, reinterpret_cast<const char*>(&receiverTid), sizeof(int), &dummy, 0);

  int tid;
  uint32_t elapsed;
  ::Receive(&tid, reinterpret_cast<char*>(&elapsed), sizeof(uint32_t));
  ::Reply(tid, &dummy, 0);
  char buffer[128];
  kit::formatString(buffer, "MessageSize=%u, %c first, avg %U ns\r\n", MessageSize, ReceiveFirst ? 'R' : 'S',
                    (uint64_t)elapsed * 1000 / REPETITIONS);
  Uart::syncPrint(buffer);
}

template <int... MessageSize> void doTest() { ((testPair<MessageSize, true>(), testPair<MessageSize, false>()), ...); }

} // namespace

namespace k2 {
void perfTestSpawner() { doTest<4, 64, 256>(); }
} // namespace k2
