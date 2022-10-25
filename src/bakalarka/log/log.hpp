#pragma once

#include "colors.hpp"
#include "Logger.hpp"
#include <sstream>

#define INIT_MODULE(MODULE) Logger lg(#MODULE);

#define LOG(msg, color) \
	do { \
		std::stringstream ss; \
		ss << msg; \
		lg.log(__func__, getpid(), ss.str().c_str(), color); \
	} while(false)

#define DBG(msg) \
	do { \
		if (lg.level() <= dbg_level::DBG) { \
			LOG(msg, GRAY); \
		}\
	} while(false)

#define INFO(msg) \
	do { \
		if (lg.level() <= dbg_level::INFO) { \
			LOG(msg, ""); \
		}\
	} while(false)

#define WARN(msg) \
	do { \
		if (lg.level() <= dbg_level::WARN) { \
			LOG(msg, YELLOW); \
		}\
	} while(false)

#define ERR(msg) \
	do { \
		if (lg.level() <= dbg_level::ERR) { \
			LOG(msg, RED); \
		} \
	} while(false)

#define FATAL(msg) \
	do { \
		if (lg.level() <= dbg_level::FATAL) { \
			LOG(msg, BACKGROUND_BRIGHT_RED); \
		}\
	} while(false)

