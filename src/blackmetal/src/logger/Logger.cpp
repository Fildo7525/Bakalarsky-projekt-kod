#include "Logger.hpp"

#include <chrono>
#include <cstdio>
#include <mutex>
#include <ostream>

Logger::Logger(const char *module, dbg_level lvl)
	: m_moduleName(module)
	, m_logFile("/home/fildo7525/Bakalarka/code/log/" + std::string(module) + __TIME__ + ".log", std::ios::app)
	, m_level(lvl)
{
}

void Logger::log(const char *codePath, pid_t pid, const char *message, const char *color)
{
	auto time = std::chrono::system_clock::now();
	std::time_t pretty_time = std::chrono::system_clock::to_time_t(time);
	std::string log_time = std::ctime(&pretty_time);
	log_time.pop_back();

	static std::mutex mut;
	{
		std::lock_guard<std::mutex> lock(mut);
		std::printf("%s [%d] %s => %s: %s\033[0;0m\n", color, pid, m_moduleName, codePath, message);
		m_logFile << log_time << "\t[" << pid << "] " << m_moduleName << " => " << codePath << ": " << message << '\n';
	}
}

dbg_level Logger::level()
{
	return m_level;
}

std::string Logger::moduleName()
{
	return m_moduleName;
}

Logger::~Logger()
{
	std::flush(m_logFile);
	m_logFile.close();
}

