#include "Logger.hpp"
#include <cstdio>
#include <ostream>

Logger::Logger(const char *module, dbg_level lvl)
	: m_moduleName(module)
	, m_level(lvl)
	, m_logFile("~/Bakalarka/workspace/log/" + std::string(module) + ".log", std::ios::app)
{
}

void Logger::log(const char *codePath, pid_t pid, const char *message, const char *color)
{
	std::printf("%s[%d] %s => %s: %s\033[0;0m\n", color, pid, m_moduleName, codePath, message);
	m_logFile << '[' << pid << "] " << m_moduleName << " => " << codePath << ": " << message << '\n';
}

dbg_level Logger::level()
{
	return m_level;
}

Logger::~Logger()
{
	std::flush(m_logFile);
	m_logFile.close();
}

