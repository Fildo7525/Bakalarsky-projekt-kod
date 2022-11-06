#pragma once

#include "colors.hpp"
#include "Logger.hpp"

#include <sstream>

#ifndef NDEBUG

#define LOG_DEFATUL_LEVEL(MODULE)             static Logger lg(#MODULE);
#define LOG_CUSTOM_LEVEL(MODULE, LOG_LEVEL)   static Logger lg(#MODULE, LOG_LEVEL);

#define GET_3RD_ARG(arg1, arg2, arg3, ...) arg3
#define PRINT_STRING_MACRO_CHOOSER(...) \
	GET_3RD_ARG(__VA_ARGS__, LOG_CUSTOM_LEVEL, LOG_DEFATUL_LEVEL, )

#define INIT_MODULE(...) PRINT_STRING_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define LOG(msg, color, log_level) \
	do { \
		std::stringstream ss; \
		ss << msg; \
		lg.log(log_level, __func__, getpid(), ss.str().c_str(), color); \
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
