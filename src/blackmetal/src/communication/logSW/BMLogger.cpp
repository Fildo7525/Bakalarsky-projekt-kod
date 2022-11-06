#include "BMLogger.hpp"
#include "log.hpp"

#include <chrono>

INIT_MODULE(BMLogger);
using namespace std::chrono_literals;

#define PORT 664

BMLogger::BMLogger()
	: rclcpp::Node("bm_logger")
	, Client()
{
	this->start(PORT, this->declare_parameter("bm_lsIP", "192.168.1.3"));

	INFO("Server created starting the timer.");
	m_timer = this->create_wall_timer(1s, [this](){
							 INFO("Check for new messages");
							 onTimerTimeoutReadSocket();
	});
}

void BMLogger::onTimerTimeoutReadSocket()
{
	std::string msg;
	this->send("");
	if (receive(msg) != bm::Status::OK) {
		FATAL("The logging message from robot could not be received");
	} else {
		INFO("BlackMetal: " << msg);
	}
}

