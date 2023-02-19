#include "Logger.hpp"

#include <mutex>
#include <thread>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <ostream>
#include <filesystem>

#include <stdio.h>

static std::mutex mut;

const std::string currentDateTime()
{
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
	std::string name = getenv("PWD") + std::string("/log/run-") + currentDateTime() + '/' + module + ".log";
	return name;
}

std::fstream Logger::m_logAllFile = std::fstream(getLoggerPath("all"), std::ios::out);

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

	std::stringstream threadID;
	threadID << std::this_thread::get_id();

	std::thread([&] {
		if (dbgLevel >= m_level) {
			std::scoped_lock<std::mutex> lk(mut);
			if (dbgLevel == dbg_level::WARN || dbgLevel == dbg_level::ERR || dbgLevel == dbg_level::FATAL) {
				std::fprintf(stderr, "%s[%d:%s] %s : %s => %s: %s\033[0;0m\n", color, pid, threadID.str().c_str(), dbgLevelToString(dbgLevel), m_moduleName, codePath, message);
			} else {
				std::printf("%s[%d:%s] %s: %s => %s: %s\033[0;0m\n", color, pid, threadID.str().c_str(), dbgLevelToString(dbgLevel), m_moduleName, codePath, message);
			}
		}
		if (!m_logFile.is_open()) {
			return;
		}

		std::stringstream ss;
		ss << log_time << "\t[" << pid << ':' << threadID.str() << "] " << dbgLevelToString(dbgLevel) << ": " << m_moduleName << " => " << codePath << ": " << message << '\n';
		{
			std::scoped_lock<std::mutex> lk(mut);
			m_logFile << ss.str();
			m_logAllFile << ss.str();
		}
	}).detach();
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

