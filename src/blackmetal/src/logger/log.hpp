#pragma once

#include "colors.hpp"
#include "Logger.hpp"

#include <sstream>

#define LOG_DEFAULT_LEVEL(MODULE)             static Logger lg(#MODULE);
#define LOG_CUSTOM_LEVEL(MODULE, LOG_LEVEL)   static Logger lg(#MODULE, LOG_LEVEL);

#define GET_3RD_ARG(arg1, arg2, arg3, ...) arg3
#define PRINT_STRING_MACRO_CHOOSER(...) \
	GET_3RD_ARG(__VA_ARGS__, LOG_CUSTOM_LEVEL, LOG_DEFAULT_LEVEL, )

/// Creates a multi instance macro. Here you can specify the module log level.
/// @see LOG_DEFAULT_LEVEL @see LOG_CUSTOM_LEVEL
#define INIT_MODULE(...) PRINT_STRING_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

/// Writes the supplied message with the log level to log file based on the module and on the stdout/stderr stream.
#define LOG(msg, color, log_level) \
	do { \
		if (lg.level() != Logger::level::OFF) { \
			std::stringstream ss; \
			ss << msg; \
			lg.log(log_level, __func__, getpid(), ss.str().c_str(), color); \
		} \
	} while(false)

/// Passes a debug log message to the registered logger. Works only in debug build.
#define DBG(msg) LOG(msg, GRAY, Logger::level::DBG)
/// Passes a info log message to the registered logger. Works only in debug build.
#define INFO(msg) LOG(msg, RESET, Logger::level::INFO)
/// Passes a warn log message to the registered logger. Works only in debug build.
#define WARN(msg) LOG(msg, YELLOW, Logger::level::WARN)
/// Passes a error log message to the registered logger. Works only in debug build.
#define ERR(msg) LOG(msg, RED, Logger::level::ERR)
/// Passes a fatal log message to the registered logger. Works only in debug build.
#define FATAL(msg) LOG(msg, BACKGROUND_BRIGHT_RED, Logger::level::FATAL)
/// Passes a success log message to the registered logger. Works only in debug build.
#define SUCCESS(msg) LOG(msg, BOLD_GREEN, Logger::level::SUCCESS)

