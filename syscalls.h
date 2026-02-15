#pragma once

inline constexpr int MAX_PRIORITY_LEVEL = 10;
enum EventId : int { TIMER1, TIMER3, UART_RX, UART_TX, CAN_IO };

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
