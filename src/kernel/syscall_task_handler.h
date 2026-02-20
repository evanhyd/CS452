#pragma once

namespace syscall_handler {

int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();

} // namespace syscall_handler
