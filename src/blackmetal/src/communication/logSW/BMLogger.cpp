#include "BMLogger.hpp"
#include "log.hpp"

#include <chrono>

INIT_MODULE(BMLogger, dbg_level::WARN);
using namespace std::chrono_literals;

#define PORT 664

BMLogger::BMLogger()
	: rclcpp::Node("bm_logger")
	, m_client()
{
	m_client->start(PORT, this->declare_parameter("bm_lsIP", "192.168.1.3"), -1);

	INFO("Server created starting the timer.");
	m_timer = this->create_wall_timer(1s, [this](){
							 DBG("Check for new messages");
							 onTimerTimeoutReadSocket();
	});
}

void BMLogger::onTimerTimeoutReadSocket()
{
	std::string msg;
	m_client->send("");

	if (m_client->receive(msg) != bm::Status::OK) {
		FATAL("The logging message from robot could not be received");
	}
	else {
		INFO("BlackMetal: " << msg);
	}
}

