#pragma once

#include <unistd.h>

#include <fstream>
#include <iostream>

/**
 * @class Logger
 * @brief Class handling all the debugging from the macros in log.hpp
 *
 * The ros2 logger does not support the functionality that is in this file.
 * The builtin logger does not define modules but writes all the logs to the same file.
 * The structure is different, too. Therefore I have created
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
	/**
	 * @brief Logging levels ordered from the most verbose to the least verbose.
	 */
	enum class level {
		DBG,
		INFO,
		WARN,
		ERR,
		FATAL,
		SUCCESS,
		OFF
	};

	/// Constructor
	explicit Logger(const char *module, level lvl = level::INFO);

	/**
	 * @brief Converts the log level to a specified string.
	 *
	 * @param level Log level to be converted.
	 * @return log level name in string type.
	 */
	static const char* dbgLevelToString(const level level);

	/**
	 * @brief Log message at a specific codePath with a colour to the screen.
	 *
	 * The same log is printed to the file named according to the module name. In the file
	 * is a timestamp at the beginning of the logging message. This method is counting with
	 * multiple threads writing to the standard output and to the same file. Thus the printing
	 * is guarded with scoped mutex.
	 *
	 * @param dbgLevel Level of the log that is to be printed.
	 * @param codePath Function name where the logging message is supported.
	 * @param pid Process printing the message.
	 * @param message Message to be printed.
	 * @param color Color of the text based on the logging level.
	 */
	void log(const level dbgLevel, const char *codePath, pid_t pid, const char *message, const char *color = "");

	/**
	 * Prints info about the class.
	 *
	 * @return The logging level for the module.
	 */
	level level();

	/**
	 * Returns the name of the module.
	 */
	std::string moduleName();

	/**
	 * Free the resources.
	 */
	~Logger();

private:
	/// Name of the module. This name is the name of the logging file, too.
	const char *m_moduleName;

	/// Stream of the logging file.
	std::fstream m_logFile;

	/// File where everything is logged.
	static std::fstream m_logAllFile;

	/// Logging level for the module.
	enum level m_level;
};

