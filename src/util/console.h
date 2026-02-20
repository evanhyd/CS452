#pragma once

#include "util/ctfmt.h"
#include "server_tasks/io_server.h"
#include "kernel/syscalls.h"

class Console {
public:
  explicit Console(int tid) : tid_{tid} {}

  template <typename... Args> void printf(kit::FormatSpec<Args...> spec, Args... args) {
    kit::printf(tid_, spec, args...);
  }

  void puts(const char* str) {
    for (; *str; ++str) {
      ::Putc(tid_, *str);
    }
  }

  void putc(char c) { ::Putc(tid_, c); }

  void moveCursor(unsigned row, unsigned col) { kit::printf(tid_, "\033[%u;%uH", row, col); }

  void clearScreen() { puts("\033[2J"); }
  void clearLine() { puts("\033[2K"); }
  void clearToEol() { puts("\033[K"); }

  void hideCursor() { puts("\033[?25l"); }
  void showCursor() { puts("\033[?25h"); }

private:
  int tid_;
};
