#pragma once

enum Priority : int { HIGH, MEDIUM, LOW, COUNT };

extern "C" {
int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();
}
