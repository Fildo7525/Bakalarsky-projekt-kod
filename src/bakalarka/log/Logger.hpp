#include <fstream>
#include <iostream>
#include <unistd.h>

enum class dbg_level {
	DBG,
	INFO,
	WARN,
	ERR,
	FATAL,
};

class Logger
{
public:
	explicit Logger(const char *module, dbg_level lvl = dbg_level::INFO);
	void log(const char *codePath, pid_t pid, const char *message, const char *color = "");
	dbg_level level();
	~Logger();

private:
	const char *m_moduleName;
	std::fstream m_logFile;
	dbg_level m_level;
};

