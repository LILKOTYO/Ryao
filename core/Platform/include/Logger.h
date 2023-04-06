#ifndef RYAO_LOGGER_H
#define RYAO_LOGGER_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/fmt/ostr.h>
#include <memory>

namespace Ryao {
    class Logger {
    public:
        static void Init();

        static std::shared_ptr<spdlog::logger>& GetLogger() {return _console; }

    private:
        static std::shared_ptr<spdlog::logger> _console;
    };

// For External Debug
#define RYAO_TRACE(...)	    Ryao::Logger::GetLogger()->trace(__VA_ARGS__)
#define RYAO_DEBUG(...)	    Ryao::Logger::GetLogger()->debug(__VA_ARGS__)
#define RYAO_INFO(...)	    Ryao::Logger::GetLogger()->info(__VA_ARGS__)
#define RYAO_WARN(...)	    Ryao::Logger::GetLogger()->warn(__VA_ARGS__)
#define RYAO_ERROR(...)	    Ryao::Logger::GetLogger()->error(__VA_ARGS__)

}

#endif