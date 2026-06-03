#pragma once

#include <cstdarg>
#include <cstdint>

namespace circle::fc_iface {

class ILogSink {
 public:
  virtual ~ILogSink() = default;

  virtual void log(const char* fmt, ...) __attribute__((format(printf, 2, 3))) = 0;

  virtual void logThrottled(uint32_t interval_ms, const char* fmt, ...)
      __attribute__((format(printf, 3, 4))) = 0;
};

class NullLogSink : public ILogSink {
 public:
  void log(const char*, ...) override {}
  void logThrottled(uint32_t, const char*, ...) override {}
};

}  // namespace circle::fc_iface
