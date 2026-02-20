#pragma once

namespace syscall_handler {

int Send(int tid, const char* message, int messageSize, char* replyBuffer, int replyBufferSize);
int Receive(int* tid, char* receiveBuffer, int receiveBufferSize);
int Reply(int tid, const char* reply, int replySize);

} // namespace syscall_handler
