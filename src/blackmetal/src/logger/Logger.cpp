#include "Logger.hpp"

#include <thread>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <mutex>
#include <ostream>
#include <filesystem>

#include <stdio.h>

/// Mutex is locked on every writing to the file or on a screen.
static std::mutex mut;

const std::string currentDateTime() {
	time_t now = time(0);
	struct tm  tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

	return buf;
}

std::string getLoggerPath(const std::string &module)
{
	std::string name = getenv("PWD") + std::string("/log/run-") + currentDateTime() + '/' + module + '-' + currentDateTime() + ".log";
	return name;
}

Logger::Logger(const char *module, dbg_level lvl)
	: m_moduleName(module)
	, m_logFile()
	, m_level(lvl)
{
	std::string location = getenv("PWD") + std::string("/log/run-") + currentDateTime();
	if (!std::filesystem::exists(location)) {
		std::filesystem::create_directory(location);
		std::printf("\nCreating the location %s\n", location.c_str());
	}
	m_logFile.open( getLoggerPath(m_moduleName), std::ios::out);
	if (!m_logFile.is_open()) {
		std::cerr << "The log file " << getLoggerPath(m_moduleName) << " could not be opened. The logs will only be visible on the screen.\n";
	}
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

	std::stringstream s;
	s << std::this_thread::get_id();

	if (dbgLevel >= m_level) {
		if (dbgLevel == dbg_level::WARN || dbgLevel == dbg_level::ERR || dbgLevel == dbg_level::FATAL) {
			std::lock_guard<std::mutex> lk(mut);
			std::fprintf(stderr, "%s%s: [%d:%s] %s => %s: %s\033[0;0m\n", dbgLevelToString(dbgLevel), color, pid, s.str().c_str(), m_moduleName, codePath, message);
		} else {
			std::lock_guard<std::mutex> lk(mut);
			std::printf("%s%s: [%d:%s] %s => %s: %s\033[0;0m\n", dbgLevelToString(dbgLevel), color, pid, s.str().c_str(), m_moduleName, codePath, message);
		}
	}
	if (!m_logFile.is_open()) {
		return;
	}
	{
		std::lock_guard<std::mutex> lk(mut);
		m_logFile << log_time << "\t[" << pid << ":" << s.str() << "] " << dbgLevelToString(dbgLevel) << ": " << m_moduleName << " => " << codePath << ": " << message << '\n';
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

