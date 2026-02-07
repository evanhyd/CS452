#pragma once

enum Priority : int { HIGHEST, HIGH, MEDIUM, LOW, LOWEST, COUNT };
enum EventId : int { TIMER1 = 97, TIMER3 = 99 };

extern "C" {
int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();

int Send(int tid, const char* message, int messageSize, char* replyBuffer, int replyBufferSize);
int Receive(int* tid, char* receiveBuffer, int receiveBufferSize);
int Reply(int tid, const char* reply, int replySize);

int RegisterAs(const char* name);
int WhoIs(const char* name);

int AwaitEvent(int eventId);

int Time(int tid);
int Delay(int tid, int ticks);
int DelayUntil(int tid, int ticks);
}
