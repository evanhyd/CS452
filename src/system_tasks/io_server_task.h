#pragma once

namespace io_server {

inline constexpr const char* IO_SERVER_NAME = "io_server";

void ioServerTask();

} // namespace io_server

extern "C" {
int Getc(int tid);
int Putc(int tid, unsigned char ch);
}
