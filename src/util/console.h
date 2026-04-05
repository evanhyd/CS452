#pragma once

#include "system_tasks/io_server_task.h"
#include "util/ctfmt.h"

class Console {
public:
  explicit Console(int tid) : tid_{tid} {}

  template <typename... Args> void printf(kit::FormatSpec<Args...> spec, const Args&... args) {
    kit::printf(tid_, spec, args...);
  }

  unsigned puts(const char* str) {
    unsigned len = 0;
    for (; *str; ++str, ++len) {
      ::Putc(tid_, static_cast<unsigned char>(*str));
    }
    return len;
  }

  void putc(char c) { ::Putc(tid_, static_cast<unsigned char>(c)); }

  void putTimestamp(unsigned ticks) {
    unsigned mins = ticks / 6000;
    unsigned secs = ticks / 100 % 60;
    unsigned tenths = ticks % 100 / 10;
    printf("%02u:%02u.%u", mins, secs, tenths);
  }

  void putTicks(unsigned ticks) {
    unsigned mins = ticks / 6000;
    unsigned secs = ticks / 100 % 60;
    unsigned remTicks = ticks % 100;
    printf("%02u:%02u.%02u", mins, secs, remTicks);
  }

  void putByte(uint8_t byte) {
    static constexpr char hex[] = "0123456789ABCDEF";
    putc(hex[(byte >> 4) & 0x0F]);
    putc(hex[byte & 0x0F]);
  }

  void moveCursor(unsigned row, unsigned col) { kit::printf(tid_, "\033[%u;%uH", row, col); }

  void nextLine() { puts("\033[E"); }
  void prevLine() { puts("\033[F"); }

  void clearScreen() { puts("\033[2J"); }
  void clearLine() { puts("\033[2K"); }
  void clearToEol() { puts("\033[K"); }

  void hideCursor() { puts("\033[?25l"); }
  void showCursor() { puts("\033[?25h"); }

  void enableMouse() { puts("\033[?1006h\033[?1000h"); }

private:
  int tid_;
};
