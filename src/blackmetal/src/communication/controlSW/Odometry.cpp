#include "Odometry.hpp"
#include "controlSW/BlackMetal.hpp"
#include "log.hpp"
#include <chrono>
#include <mutex>
#include <variant>

std::mutex odometryMutex;

INIT_MODULE(Odometry);

Odometry::Odometry(BlackMetal &controlSoftware)
	: m_controlSoftware(controlSoftware)
{
	std::lock_guard<std::mutex> lock(odometryMutex);
	m_timer = m_controlSoftware.create_wall_timer(
			std::chrono::milliseconds(100),
			[this]() {
				this->execute();
			}
	);
	WARN("Timer initialized");
}

void Odometry::execute()
{
	std::variant<bm::Status, std::string> temp;
	{
		std::lock_guard<std::mutex> lock(odometryMutex);
		temp = m_controlSoftware.execute(bm::Command::GET_LR_WHEEL_VELOCITY);
	}
	std::string wheelSpeed;

	if (std::get_if<std::string>(&temp)) {
		wheelSpeed = std::get<std::string>(temp);
		DBG("Message received: " << wheelSpeed);
	}
	speed wheels = obtainWheelSpeeds(wheelSpeed);

	std::lock_guard<std::mutex> lock(odometryMutex);
	m_leftWheel = wheels.leftWheel;
	m_rightWheel = wheels.rightWheel;
}

Odometry::speed Odometry::obtainWheelSpeeds(const std::string &jsonMessage)
{
	// The structure will arrive in a wonnabe json format
	// {"LeftWheelSpeed"=%ld "RightWheelSpeed"=%ld}
	auto lws_start = jsonMessage.find_first_of('=');
	WARN("lws_start: " << lws_start);
	auto lws_end = jsonMessage.find_first_of(' ');
	WARN("lws_end: " << lws_end);
	long lws = std::stol(jsonMessage.substr(lws_start, lws_end));
	WARN("lws: " << lws);

	auto lrs_start = jsonMessage.find_last_of('=');
	WARN("lrs_start: " << lrs_start);
	auto lrs_end = jsonMessage.find_last_of('}');
	WARN("lrs_end: " << lrs_end);
	long lrs = std::stol(jsonMessage.substr(lrs_start, lrs_end));
	WARN("lrs: " << lrs);

	return {lws, lrs};
}

long Odometry::leftWheelSpeed() const
{
	std::lock_guard<std::mutex> lock(odometryMutex);
	return m_leftWheel;
}

long Odometry::rightWheelSpeed() const
{
	std::lock_guard<std::mutex> lock(odometryMutex);
	return m_rightWheel;
}

