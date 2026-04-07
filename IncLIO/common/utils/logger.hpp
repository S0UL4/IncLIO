#ifndef INCLIO_LOGGER_HPP
#define INCLIO_LOGGER_HPP

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace IncLIO {

// Convenience macros that use the default spdlog logger.
// Usage:  INCLIO_INFO("Processing scan {}", scan_id);

#define INCLIO_TRACE(...)    SPDLOG_TRACE(__VA_ARGS__)
#define INCLIO_DEBUG(...)    SPDLOG_DEBUG(__VA_ARGS__)
#define INCLIO_INFO(...)     SPDLOG_INFO(__VA_ARGS__)
#define INCLIO_WARN(...)     SPDLOG_WARN(__VA_ARGS__)
#define INCLIO_ERROR(...)    SPDLOG_ERROR(__VA_ARGS__)
#define INCLIO_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

// Call once at startup to configure the global logger
inline void InitLogger(spdlog::level::level_enum level = spdlog::level::info) {
    spdlog::set_level(level);
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
}

} // namespace IncLIO

#endif // INCLIO_LOGGER_HPP
