#include "Logger.h"

namespace Ryao {
    std::shared_ptr<spdlog::logger> Logger::_console;

    void Logger::Init()
	{
		_console = spdlog::stdout_color_mt("Ryao Console");
		_console->set_pattern("%^[%T | %n]: %v %$");
		_console->set_level(spdlog::level::trace);
		_console->trace("Logger created !");
	}
}