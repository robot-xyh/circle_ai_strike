#pragma once

#include <sstream>
#include <string>
#include <utility>

namespace circle::bf {

enum class LogLevel {
  Debug,
  Info,
  Warn,
  Error,
  Off,
};

enum class LogColorMode {
  Auto,
  Always,
  Never,
};

bool parseLogLevel(const std::string& name, LogLevel* level);
bool parseLogColorMode(const std::string& name, LogColorMode* mode);
void setLogLevel(LogLevel level);
void setLogColorMode(LogColorMode mode);
void configureLogger(const std::string& level_name,
                     const std::string& color_mode_name);
bool logEnabled(LogLevel level);
void log(LogLevel level, const std::string& message);

template <typename... Args>
void logMessage(LogLevel level, Args&&... args) {
  if (!logEnabled(level)) {
    return;
  }
  std::ostringstream oss;
  (oss << ... << std::forward<Args>(args));
  log(level, oss.str());
}

template <typename... Args>
void logDebug(Args&&... args) {
  logMessage(LogLevel::Debug, std::forward<Args>(args)...);
}

template <typename... Args>
void logInfo(Args&&... args) {
  logMessage(LogLevel::Info, std::forward<Args>(args)...);
}

template <typename... Args>
void logWarn(Args&&... args) {
  logMessage(LogLevel::Warn, std::forward<Args>(args)...);
}

template <typename... Args>
void logError(Args&&... args) {
  logMessage(LogLevel::Error, std::forward<Args>(args)...);
}

}  // namespace circle::bf
