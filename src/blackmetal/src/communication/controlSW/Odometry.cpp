#include "Odometry.hpp"
#include "controlSW/BlackMetal.hpp"
#include "log.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <variant>
#include <thread>

std::mutex g_odometryMutex;
std::mutex g_robotLocationMutex;
// TIP: The 3 second interval is just for debugging
const std::chrono::milliseconds g_pollTime(3000);

INIT_MODULE(Odometry);

Odometry::Odometry(BlackMetal &controlSoftware)
	: m_controlSoftware(controlSoftware)
	, m_coordination({0, 0, 0})
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	m_timer = m_controlSoftware.create_wall_timer(
			std::chrono::milliseconds(g_pollTime),
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
		std::lock_guard<std::mutex> lock(g_odometryMutex);
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

	{
		std::lock_guard<std::mutex> lock(g_odometryMutex);
		m_leftWheel = wheels.leftWheel;
		m_rightWheel = wheels.rightWheel;
	}

	std::thread(std::bind(&Odometry::changeRobotLocation, this, std::placeholders::_1), std::move(wheels)).detach();
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
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	return m_leftWheel;
}

long Odometry::rightWheelSpeed() const
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
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
void Odometry::changeRobotLocation(Speed &&speed)
{
	double dt = g_pollTime.count(); // TODO: Plus the time of sending and receiving a request
	double angularVelocity = (speed.rightWheel - speed.leftWheel) / m_controlSoftware.chassisLength();
	double speedInCenterOfGravity = (speed.rightWheel + speed.leftWheel) / 2.0;
	// auto icr = m_controlSoftware.chassisLength() / 2.0 * (speed.rightWheel + speed.leftWheel) / (speed.rightWheel - speed.leftWheel);

	auto vx = speedInCenterOfGravity *std::cos(m_coordination.angle);
	auto vy = speedInCenterOfGravity *std::sin(m_coordination.angle);

	auto dxt = vx * dt;
	auto dyt = vy * dt;
	auto changOfAngleInTime = angularVelocity * dt;

	{
		std::lock_guard<std::mutex> lock(g_robotLocationMutex);
		m_coordination.x += dxt;
		m_coordination.y += dyt;
		m_coordination.angle += changOfAngleInTime;
	}
}

