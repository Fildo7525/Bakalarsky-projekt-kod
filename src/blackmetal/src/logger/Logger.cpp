#include "Logger.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <mutex>
#include <ostream>
#include <filesystem>

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

std::string getLoggerPath(const std::string &module)
{
	std::string name = "~/Bakalarka/code/log/" + module + '-' + currentDateTime() + ".log";
	return name;
}

Logger::Logger(const char *module, dbg_level lvl)
	: m_moduleName(module)
	, m_logFile(getLoggerPath(module), std::ios::app)
	, m_level(lvl)
{
	std::string location = getenv("PWD") + std::string("/log/");
	if (!std::filesystem::exists(location)) {
		system("mkdir ./log/");
	}
	m_logFile.open( location + std::string(module) + __TIME__ + ".log", std::ios::app);
}

const char* Logger::dbgLevelToString(const dbg_level level)
{
	switch (level) {
		case dbg_level::DBG:
			return "DBG";
		case dbg_level::INFO:
			return "INFO";
		case dbg_level::WARN:
			return "WARN";
		case dbg_level::ERR:
			return "ERR";
		case dbg_level::FATAL:
			return "FATAL";
		case dbg_level::SUCCESS:
			return "SUCCESS";
		default:
			return "";
	}
}

void Logger::log(const dbg_level dbgLevel, const char *codePath, pid_t pid, const char *message, const char *color)
{
	auto time = std::chrono::system_clock::now();
	std::time_t pretty_time = std::chrono::system_clock::to_time_t(time);
	std::string log_time = std::ctime(&pretty_time);
	log_time.pop_back();

	std::mutex screenMutex;
	std::mutex fileMutex;
	if (dbgLevel >= m_level) {
		std::lock_guard<std::mutex> lock(screenMutex);
		if (dbgLevel == dbg_level::WARN || dbgLevel == dbg_level::ERR || dbgLevel == dbg_level::FATAL) {
			std::fprintf(stderr, "%s%s: [%d] %s => %s: %s\033[0;0m\n", dbgLevelToString(dbgLevel), color, pid, m_moduleName, codePath, message);
		} else {
			std::printf("%s%s: [%d] %s => %s: %s\033[0;0m\n", dbgLevelToString(dbgLevel), color, pid, m_moduleName, codePath, message);
		}
	}
	{
		std::lock_guard<std::mutex> lock(fileMutex);
		m_logFile << log_time << "\t[" << pid << "] " << dbgLevelToString(dbgLevel) << ": " << m_moduleName << " => " << codePath << ": " << message << '\n';
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

