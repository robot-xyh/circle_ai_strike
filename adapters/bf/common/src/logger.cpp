#include "circle/bf/logger.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>

#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace circle::bf {

namespace {

std::atomic<int>& minLogLevelStorage() {
  static std::atomic<int> level{static_cast<int>(LogLevel::Info)};
  return level;
}

std::atomic<int>& colorModeStorage() {
  static std::atomic<int> mode{static_cast<int>(LogColorMode::Auto)};
  return mode;
}

std::string normalized(std::string in) {
  std::transform(in.begin(), in.end(), in.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return in;
}

const char* levelName(LogLevel level) {
  switch (level) {
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Error:
      return "ERROR";
    case LogLevel::Off:
      return "OFF";
  }
  return "INFO";
}

const char* levelColor(LogLevel level) {
  switch (level) {
    case LogLevel::Debug:
      return "\033[2m";
    case LogLevel::Info:
      return "";
    case LogLevel::Warn:
      return "\033[33m";
    case LogLevel::Error:
      return "\033[1;31m";
    case LogLevel::Off:
      return "";
  }
  return "";
}

bool streamIsTty(std::ostream& out) {
#if defined(_WIN32)
  (void)out;
  return false;
#else
  if (&out == &std::cerr) {
    return isatty(STDERR_FILENO) != 0;
  }
  return isatty(STDOUT_FILENO) != 0;
#endif
}

bool shouldColor(std::ostream& out) {
  const auto mode =
      static_cast<LogColorMode>(colorModeStorage().load(std::memory_order_relaxed));
  switch (mode) {
    case LogColorMode::Always:
      return true;
    case LogColorMode::Never:
      return false;
    case LogColorMode::Auto:
      return streamIsTty(out);
  }
  return false;
}

std::string timestampNow() {
  const auto now = std::chrono::system_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;
  const std::time_t t = std::chrono::system_clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.'
      << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

std::mutex& logMutex() {
  static std::mutex mu;
  return mu;
}

}  // namespace

bool parseLogLevel(const std::string& name, LogLevel* level) {
  const std::string v = normalized(name);
  if (v == "debug") {
    *level = LogLevel::Debug;
    return true;
  }
  if (v == "info") {
    *level = LogLevel::Info;
    return true;
  }
  if (v == "warn" || v == "warning") {
    *level = LogLevel::Warn;
    return true;
  }
  if (v == "error" || v == "err") {
    *level = LogLevel::Error;
    return true;
  }
  if (v == "off" || v == "none" || v == "silent") {
    *level = LogLevel::Off;
    return true;
  }
  return false;
}

bool parseLogColorMode(const std::string& name, LogColorMode* mode) {
  const std::string v = normalized(name);
  if (v == "auto") {
    *mode = LogColorMode::Auto;
    return true;
  }
  if (v == "always" || v == "on" || v == "true") {
    *mode = LogColorMode::Always;
    return true;
  }
  if (v == "never" || v == "off" || v == "false") {
    *mode = LogColorMode::Never;
    return true;
  }
  return false;
}

void setLogLevel(LogLevel level) {
  minLogLevelStorage().store(static_cast<int>(level), std::memory_order_relaxed);
}

void setLogColorMode(LogColorMode mode) {
  colorModeStorage().store(static_cast<int>(mode), std::memory_order_relaxed);
}

void configureLogger(const std::string& level_name,
                     const std::string& color_mode_name) {
  LogLevel level = LogLevel::Info;
  if (parseLogLevel(level_name, &level)) {
    setLogLevel(level);
  }

  LogColorMode mode = LogColorMode::Auto;
  if (parseLogColorMode(color_mode_name, &mode)) {
    setLogColorMode(mode);
  }
}

bool logEnabled(LogLevel level) {
  const int min_level = minLogLevelStorage().load(std::memory_order_relaxed);
  return min_level < static_cast<int>(LogLevel::Off) &&
         static_cast<int>(level) >= min_level;
}

void log(LogLevel level, const std::string& message) {
  if (!logEnabled(level)) {
    return;
  }
  std::lock_guard<std::mutex> lk(logMutex());
  std::ostream& out =
      (level == LogLevel::Warn || level == LogLevel::Error) ? std::cerr
                                                            : std::cout;
  out << '[' << timestampNow() << "] ";
  const char* color = levelColor(level);
  if (shouldColor(out) && color[0] != '\0') {
    out << color << '[' << levelName(level) << "]\033[0m ";
  } else {
    out << '[' << levelName(level) << "] ";
  }
  out << message << '\n';
}

}  // namespace circle::bf
