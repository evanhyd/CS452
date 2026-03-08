#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>
#include <utility>

#include "kernel/devices/uart.h"
#include "system_tasks/io_server_task.h"

namespace kit {

class Sink {
  alignas(void*) char storage[16];
  void (*invoke)(void*, char);

public:
  template <typename T, typename D = std::decay_t<T>>
    requires(sizeof(D) <= sizeof(storage)) && std::is_trivially_copyable_v<D> && requires(D& d, char c) {
      { d(c) };
    }
  static constexpr Sink make(T&& obj) {
    Sink s;
    new (s.storage) D(std::forward<T>(obj));
    s.invoke = [](void* data, char c) { (*static_cast<D*>(data))(c); };
    return s;
  }

  void put(char c) { invoke(storage, c); }

  template <typename T> T& get() { return *reinterpret_cast<T*>(storage); }
};

namespace internal {

template <typename T>
concept CStr = std::is_same_v<T, const char*> || std::is_same_v<T, char*>;

enum class ArgType { Integer, String, Char };
template <typename T> consteval ArgType getArgType() {
  if constexpr (std::is_same_v<T, char>) {
    return ArgType::Char;
  } else if constexpr (std::is_integral_v<T>) {
    return ArgType::Integer;
  } else if constexpr (CStr<T>) {
    return ArgType::String;
  } else if constexpr (std::is_convertible_v<T, std::string_view>) {
    return ArgType::String;
  } else {
    static_assert(false, "Unsupported argument type");
  }
}

struct FormatContext {
  int width;
  char spec;
  char pad;
};

using FormatterFunc = void (*)(Sink&, const void*, FormatContext);

struct Arg {
  const void* ptr;
  FormatterFunc func;
};

template <std::integral T>
  requires(!std::is_same_v<T, bool>)
void formatInt(Sink& sink, const void* ptr, FormatContext ctx) {
  T value = *static_cast<const T*>(ptr);
  bool hex = ctx.spec == 'x' || ctx.spec == 'X';
  bool upper = ctx.spec == 'X';
  bool negative = false;
  using U = std::make_unsigned_t<T>;
  if constexpr (std::is_signed_v<T>) {
    if (!hex && value < 0) {
      negative = true;
      value = static_cast<T>(static_cast<U>(-(value + 1)) + 1);
    }
  }
  U uval = static_cast<U>(value);
  U base = hex ? 16 : 10;
  const char* digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
  char buf[24];
  int pos = 0;
  if (uval == 0) {
    buf[pos++] = '0';
  } else {
    while (uval > 0) {
      buf[pos++] = digits[uval % base];
      uval /= base;
    }
  }
  int len = pos + negative;
  if (ctx.pad == '0' && negative) {
    sink.put('-');
    negative = false;
  }
  while (ctx.width > len) {
    sink.put(ctx.pad);
    --ctx.width;
  }
  if (negative) {
    sink.put('-');
  }
  while (pos > 0) {
    sink.put(buf[--pos]);
  }
}

template <CStr T> void formatStr(Sink& sink, const void* ptr, FormatContext ctx) {
  const char* str = *static_cast<const char* const*>(ptr);
  if (!str) {
    str = "(null)";
  }
  int len = 0;
  while (str[len]) {
    len++;
  }
  while (ctx.width > len) {
    sink.put(ctx.pad);
    --ctx.width;
  }
  while (*str) {
    sink.put(*str++);
  }
}

template <std::convertible_to<std::string_view> T>
  requires(!CStr<T>)
void formatStr(Sink& sink, const void* ptr, FormatContext ctx) {
  std::string_view str{*static_cast<const T*>(ptr)};
  int len = static_cast<int>(str.size());
  while (ctx.width > len) {
    sink.put(ctx.pad);
    --ctx.width;
  }
  for (char c : str) {
    sink.put(c);
  }
}

inline void formatChar(Sink& sink, const void* ptr, FormatContext ctx) {
  while (ctx.width > 1) {
    sink.put(ctx.pad);
    --ctx.width;
  }
  sink.put(*static_cast<const char*>(ptr));
}

template <typename T> Arg makeArg(const T& val) {
  static constexpr ArgType type = getArgType<T>();
  if constexpr (type == ArgType::Integer) {
    if constexpr (std::is_same_v<T, bool>) {
      return Arg{&val, formatInt<uint8_t>};
    } else {
      return Arg{&val, formatInt<T>};
    }
  } else if constexpr (type == ArgType::String) {
    return Arg{&val, formatStr<T>};
  } else if constexpr (type == ArgType::Char) {
    return Arg{&val, formatChar};
  }
}

[[noreturn]] void error(const char* msg);

template <typename... Args> class FormatSpec {
  const char* fmt_;

public:
  consteval FormatSpec(const char* fmt) : fmt_{fmt} {
    constexpr size_t numArgs = sizeof...(Args);
    ArgType provided[numArgs > 0 ? numArgs : 1] = {getArgType<std::decay_t<Args>>()...};
    size_t argIdx = 0;
    for (int i = 0; fmt[i] != '\0'; ++i) {
      if (fmt[i] == '%') {
        if (fmt[i + 1] == '%') {
          ++i;
          continue;
        }
        if (argIdx >= numArgs) {
          error("Too few arguments.");
        }
        ++i;
        if (fmt[i] == '0') {
          ++i;
        }
        while (fmt[i] >= '0' && fmt[i] <= '9') {
          ++i;
        }
        ArgType expected = [spec = fmt[i]] {
          if (spec == 'd' || spec == 'i' || spec == 'u' || spec == 'x' || spec == 'X') {
            return ArgType::Integer;
          } else if (spec == 's') {
            return ArgType::String;
          } else if (spec == 'c') {
            return ArgType::Char;
          } else {
            error("Unsupported format specifier.");
          }
        }();
        if (expected != provided[argIdx]) {
          error("Argument type mismatch.");
        }
        ++argIdx;
      }
    }
    if (argIdx != numArgs) {
      error("Too many arguments.");
    }
  }

  constexpr const char* get() const { return fmt_; }
};

} // namespace internal

inline void doFormat(Sink& sink, const char* fmt, const internal::Arg* args) {
  int argIdx = 0;
  while (*fmt) {
    if (*fmt == '%') {
      ++fmt;
      if (*fmt == '%') {
        sink.put('%');
        ++fmt;
        continue;
      }
      internal::FormatContext ctx;
      ctx.pad = ' ';
      ctx.width = 0;
      if (*fmt == '0') {
        ctx.pad = '0';
        ++fmt;
      }
      while (*fmt >= '0' && *fmt <= '9') {
        ctx.width = ctx.width * 10 + *fmt - '0';
        ++fmt;
      }
      ctx.spec = *fmt;
      args[argIdx].func(sink, args[argIdx].ptr, ctx);
      ++argIdx;
      ++fmt;
      continue;
    }
    sink.put(*fmt++);
  }
}

template <typename... Args> using FormatSpec = std::type_identity_t<internal::FormatSpec<Args...>>;

template <typename... Args> void formatSink(Sink& sink, FormatSpec<Args...> spec, const Args&... args) {
  if constexpr (sizeof...(Args) > 0) {
    internal::Arg argArr[] = {internal::makeArg(args)...};
    doFormat(sink, spec.get(), argArr);
  } else {
    doFormat(sink, spec.get(), nullptr);
  }
}

template <typename... Args> void formatString(char* buffer, FormatSpec<Args...> spec, const Args&... args) {
  struct S {
    char* buf;
    void operator()(char c) { *buf++ = c; }
  };
  auto sink = Sink::make(S{buffer});
  formatSink(sink, spec, args...);
  *sink.template get<S>().buf = '\0';
}

template <typename... Args> void printf(int tid, FormatSpec<Args...> spec, const Args&... args) {
  auto sink = Sink::make([tid](char c) { ::Putc(tid, static_cast<unsigned char>(c)); });
  formatSink(sink, spec, args...);
}

template <typename... Args> void syncPrintf(FormatSpec<Args...> spec, const Args&... args) {
  auto sink = Sink::make(Uart::syncPutc);
  formatSink(sink, spec, args...);
}

} // namespace kit
