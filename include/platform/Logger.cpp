#include "Logger.h"

namespace Ryao {
    std::shared_ptr<spdlog::logger> Logger::m_console;

    void Logger::Init()
	{
		m_console = spdlog::stdout_color_mt("Ryao Console");
		m_console->set_pattern("%^[%T | %n]: %v %$");
		m_console->set_level(spdlog::level::trace);
		m_console->trace("Logger created !");
	}
}