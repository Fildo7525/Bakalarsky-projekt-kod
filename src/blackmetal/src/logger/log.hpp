#pragma once

#include "colors.hpp"
#include "Logger.hpp"

#include <sstream>

#ifndef NDEBUG

#define INIT_MODULE(MODULE) static Logger lg(#MODULE);

#define LOG(msg, color, log_level) \
	do { \
		if (lg.level() <= log_level) { \
			std::stringstream ss; \
			ss << msg; \
			lg.log(log_level, __func__, getpid(), ss.str().c_str(), color); \
		} \
	} while(false)

#define DBG(msg) LOG(msg, GRAY, dbg_level::DBG)
#define INFO(msg) LOG(msg, RESET, dbg_level::INFO)
#define WARN(msg) LOG(msg, YELLOW, dbg_level::WARN)
#define ERR(msg) LOG(msg, RED, dbg_level::ERR)
#define FATAL(msg) LOG(msg, BACKGROUND_BRIGHT_RED, dbg_level::FATAL)
/// Passes a success log message to the registered logger. Works only in debug build.
#define SUCCESS(msg) LOG(msg, BOLD_GREEN, dbg_level::SUCCESS)

#else

#define INIT_MODULE(MODULE) ;

#define DBG(msg)  ;
#define INFO(msg) ;
#define WARN(msg) ;
#define ERR(msg)  ;
#define FATAL(msg) ;
/// Macro is defined only in debug mode.
#define SUCCESS(msg) ;

#endif
