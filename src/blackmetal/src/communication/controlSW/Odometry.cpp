#include "Odometry.hpp"
#include "controlSW/BlackMetal.hpp"
#include "log.hpp"
#include <chrono>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <variant>

std::mutex odometryMutex;

INIT_MODULE(Odometry);

Odometry::Odometry(BlackMetal &controlSoftware)
	: m_controlSoftware(controlSoftware)
{
	std::lock_guard<std::mutex> lock(odometryMutex);
	m_timer = m_controlSoftware.create_wall_timer(
			std::chrono::milliseconds(3000),
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
	evalReturnState(wheelSpeed);

	m_controlSoftware.send("");
	m_controlSoftware.receive(wheelSpeed);
	Speed wheels = obtainWheelSpeeds(wheelSpeed);
	INFO("Obtained speeds are " << wheels.leftWheel << " and " << wheels.rightWheel);

	std::lock_guard<std::mutex> lock(odometryMutex);
	m_leftWheel = wheels.leftWheel;
	m_rightWheel = wheels.rightWheel;
}

Odometry::Speed Odometry::obtainWheelSpeeds(const std::string &jsonMessage)
{
	// The structure will arrive in a wannabe json format
	// {"LeftWheelSpeed"=%ld "RightWheelSpeed"=%ld}
	long lws, rws;
	try {
		// WARN(nextAttempt);
		// m_controlSoftware.receive(nextAttempt);

		// WARN("Next attempt: " << jsonMessage);
		auto lws_start = jsonMessage.find_first_of('=') + 1;
		auto lws_end = jsonMessage.find_first_of(' ');
		// WARN("First substring: " << jsonMessage.substr(lws_start, lws_end));
		lws = std::stol(jsonMessage.substr(lws_start, lws_end));
		// WARN("Long of first substing: " << lws);

		auto rws_start = jsonMessage.find_last_of('=') + 1;
		auto rws_end = jsonMessage.find_last_of('}');
		// WARN("Second substring: " << jsonMessage.substr(rws_start, rws_end));
		rws = std::stol(jsonMessage.substr(rws_start, rws_end));
		// WARN("Long of second substing: " << rws);
	} catch (std::out_of_range &e) {
		std::string nextAttempt;
		m_controlSoftware.receive(nextAttempt);
		return obtainWheelSpeeds(nextAttempt);
	} catch (std::exception &e) {
		ERR(e.what());
		return {0,0};
	}

	return {lws, rws};
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

bm::Status Odometry::evalReturnState(const std::string &returnJson)
{
	if (returnJson.find("RECIEVE_OK") == std::string::npos) {
		WARN("The robot buffer is full. The send data will not be used: " << returnJson);
		return bm::Status::FULL_BUFFER;
	}

	SUCCESS(returnJson);
	return bm::Status::OK;
}
