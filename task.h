#pragma once

extern "C" {
int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();
int Send(int tid, const char* message, int messageSize, char* replyBuffer, int replyBufferSize);
int Receive(int* tid, char* receiveBuffer, int receiveBufferSize);
int Reply(int tid, const char* reply, int replySize);
}
