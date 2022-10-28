#include "BMLogger.hpp"
#include "log.hpp"

#include <chrono>

INIT_MODULE(BMLogger);
using namespace std::chrono_literals;

#define PORT 664

BMLogger::BMLogger()
	: rclcpp::Node("bm_logger")
	, Client(PORT, "192.168.1.3")
{
	this->create_wall_timer(1s, [this](){ onTimerTimeoutReadSocket(); });
}

void BMLogger::onTimerTimeoutReadSocket()
{
	INFO("BlackMetal: " << recieve());
}

