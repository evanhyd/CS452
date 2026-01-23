#pragma once

struct StackContext;

extern "C" [[noreturn]] void syscallEntry(StackContext* userStack);

extern "C" [[noreturn]] void placeholderEntry();
