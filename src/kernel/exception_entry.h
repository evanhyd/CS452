#pragma once

struct StackContext;

extern "C" {
[[noreturn]] void syscallEntry(StackContext* userStack);
[[noreturn]] void irqEntry(StackContext* userStack);
[[noreturn]] void placeholderEntry(int group, int entry);
}
