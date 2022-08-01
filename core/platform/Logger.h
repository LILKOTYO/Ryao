#ifndef RYAO_LOGGER_H
#define RYAO_LOGGER_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/fmt/ostr.h>

namespace Ryao {
    class Logger {
    public:
        static void Init();

        static std::shared_ptr<spdlog::logger>& GetLogger() {return m_console; }

    private:
        static std::shared_ptr<spdlog::logger> m_console;
    };

// For External Debug
#define RYAO_TRACE(...)	    Ryao::Logger::GetLogger()->trace(__VA_ARGS__)
#define RYAO_DEBUG(...)	    Ryao::Logger::GetLogger()->debug(__VA_ARGS__)
#define RYAO_INFO(...)	    Ryao::Logger::GetLogger()->info(__VA_ARGS__)
#define RYAO_WARN(...)	    Ryao::Logger::GetLogger()->warn(__VA_ARGS__)
#define RYYAO_ERROR(...)	Ryao::Logger::GetLogger()->error(__VA_ARGS__)

// // For Internal Debug
// #define RYAO_TRACE_IN(...)	    Logger::GetLogger()->trace(__VA_ARGS__)
// #define RYAO_DEBUG_IN(...)	    Logger::GetLogger()->debug(__VA_ARGS__)
// #define RYAO_INFO_IN(...)	    Logger::GetLogger()->info(__VA_ARGS__)
// #define RYAO_WARN_IN(...)	    Logger::GetLogger()->warn(__VA_ARGS__)
// #define RYYAO_ERROR_IN(...)	    Logger::GetLogger()->error(__VA_ARGS__)
}

#endif