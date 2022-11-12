#pragma once

#include <fstream>
#include <iostream>

#include <unistd.h>

/**
 * @brief Logging levels ordered from the most verbous to the least verbous.
 */
enum class dbg_level {
	DBG,
	INFO,
	WARN,
	ERR,
	FATAL,
	SUCCESS
};

/**
 * @class Logger
 * @brief Class handelling all the debugging from the macros in @file log.hpp
 *
 * The ros2 logger does not support the functionality that is in this file.
 * The buildint logger would have to defile the modules on every write and will dump
 * all the logs to the same file. The structure is different, too. Therefor I have created 
 * a Logger class that suits my needs.
 *
 * TODO: The logging level could be changed during the runtime.
 * There could be a file containing names of all the modules with a default logging level.
 * The logging level could be taken from the file on every print or just on initialization.
 *
 * WARN: This will slower the program significantly.
 * This would be better with QT5 and FileWatcher.
 */
class Logger
{
public:
	/// Constructor
	explicit Logger(const char *module, dbg_level lvl = dbg_level::INFO);

	/**
	 * @brief Converts the log level to a specified string.
	 *
	 * @param level Log level to be converted.
	 * @return log level name in string type.
	 */
	static const char* dbgLevelToString(const dbg_level level);

	/**
	 * @brief Log message at a specific codePath with a colour to the screen.
	 *
	 * The same log is printed to the file named according to the module name. In the file
	 * is a timestamp at the beginnging of the logging message. This method is counting with
	 * multiple threads writing to the standard output and to the same file. Thus the 
	 * printing is guarded with scoped mutex.
	 *
	 * @param codePath Function name where the logging message is supported.
	 * @param pid Process printing the message.
	 * @param message Message to be printed.
	 * @param color Color of the text based on the logging level.
	 */
	void log(const dbg_level dbgLevel, const char *codePath, pid_t pid, const char *message, const char *color = "");

	/**
	 * @brief Prints info about the class.
	 *
	 * @return The logging level for the module.
	 */
	dbg_level level();

	/**
	 * @brief Returns the name of the module.
	 */
	std::string moduleName();

	/**
	 * @brief Free the resources.
	 */
	~Logger();

private:
	/// Name of the module. This name is the name of the logging file, too.
	const char *m_moduleName;
	/// Stream of the logging file.
	std::fstream m_logFile;
	/// Logging level fot the module.
	dbg_level m_level;
};

