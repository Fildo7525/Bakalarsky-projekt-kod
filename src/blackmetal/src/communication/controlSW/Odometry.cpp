#include "Odometry.hpp"

#include "controlSW/BlackMetal.hpp"
#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <variant>

/// Defined by the BlackMetal source code.
#define MAX_WHEEL_SPEED 1000

std::mutex g_odometryMutex;
std::mutex g_robotLocationMutex;
// TIP: The 3 second interval is just for debugging
constexpr std::chrono::milliseconds g_pollTime(3000);

using namespace std::placeholders;

INIT_MODULE(Odometry, dbg_level::DBG);

Odometry::Odometry(std::shared_ptr<BlackMetal> controlSoftware)
	: m_controlSoftware(controlSoftware)
	, m_coordination({0, 0, 0})
{
	m_robotSpeedReceiver = std::thread(
		[this] () {
			while (true) {
				std::this_thread::sleep_for(g_pollTime);
				execute();
			}
		}
	);
	INFO("Timer initialized");
}

void Odometry::execute()
{
	static Speed lastValue{std::numeric_limits<long>::max(), std::numeric_limits<long>::max()};
	std::variant<bm::Status, std::string> temp;
	{
		std::lock_guard<std::mutex> lock(g_odometryMutex);
		temp = m_controlSoftware->execute(bm::Command::GET_LR_WHEEL_VELOCITY);
	}

	std::string wheelSpeed;
	if (std::get_if<std::string>(&temp)) {
		wheelSpeed = std::get<std::string>(temp);
		DBG("Message received: " << wheelSpeed);
	}
	evalReturnState(wheelSpeed);

	Speed wheels;
	TIC;
	// The send and receive methos are thread safe.
	m_controlSoftware->send("");
	m_controlSoftware->receive(wheelSpeed);

	wheels = obtainWheelSpeeds(wheelSpeed);

	if (wheels.leftWheel != lastValue.leftWheel || wheels.rightWheel != lastValue.rightWheel) {
		INFO("Obtained speeds are " << wheels.leftWheel << " and " << wheels.rightWheel);
		lastValue = wheels;
	}

	{
		std::lock_guard<std::mutex> lock(g_odometryMutex);
		m_velocity.leftWheel = wheels.leftWheel;
		m_velocity.rightWheel = wheels.rightWheel;
	}
	TOC;
	double elapsedTime = Stopwatch::lastStoppedTime();

	std::thread(std::bind(&Odometry::changeRobotLocation, this, _1, _2), std::move(wheels), std::move(elapsedTime)).detach();
}

Odometry::Speed Odometry::obtainWheelSpeeds(const std::string &jsonMessage) const
{
	// The structure will arrive in a wannabe json format
	// {"LeftWheelSpeed"=%ld "RightWheelSpeed"=%ld}
	// TODO: check if the try-catch block is necessary
	long lws, rws;
	try {
		DBG("Next attempt: " << jsonMessage);
		auto lws_start = jsonMessage.find_first_of('=') + 1;
		auto lws_end = jsonMessage.find_first_of(' ');
		lws = std::stol(jsonMessage.substr(lws_start, lws_end));

		auto rws_start = jsonMessage.find_last_of('=') + 1;
		auto rws_end = jsonMessage.find_last_of('}');
		rws = std::stol(jsonMessage.substr(rws_start, rws_end));
	} catch (std::out_of_range &e) {
		DBG("Attempting to receive the json on next read");
		std::string nextAttempt;
		m_controlSoftware->receive(nextAttempt);
		return obtainWheelSpeeds(nextAttempt);
	} catch (std::exception &e) {
		ERR(e.what());
		return {0,0};
	}

	lws = lws > MAX_WHEEL_SPEED ? 0 : lws;
	rws = rws > MAX_WHEEL_SPEED ? 0 : rws;

	return {lws, rws};
}

long Odometry::leftWheelSpeed() const
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	return m_velocity.leftWheel;
}

long Odometry::rightWheelSpeed() const
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	return m_velocity.rightWheel;
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

void Odometry::changeRobotLocation(Speed &&speed, long double &&elapsedTime)
{
	// The poll time is in milliseconds while th elapsedTime is in microseconds.
	long double dt = (g_pollTime.count() + elapsedTime/1'000.) / 1'000.;
	DBG("dt: " << dt);
	double angularVelocity = (speed.rightWheel - speed.leftWheel) / m_controlSoftware->chassisLength();
	DBG("Angular velocity: " << angularVelocity);
	double speedInCenterOfGravity = (speed.rightWheel + speed.leftWheel) / 2.0;
	DBG("CoG velocity: " << speedInCenterOfGravity);
	// auto icr = m_controlSoftware->chassisLength() / 2.0 * (speed.rightWheel + speed.leftWheel) / (speed.rightWheel - speed.leftWheel);

	double vx;
	double vy;
	{
		std::lock_guard<std::mutex> lock(g_robotLocationMutex);
		vx = speedInCenterOfGravity * std::cos(m_coordination.angle);
		vy = speedInCenterOfGravity * std::sin(m_coordination.angle);
	}

	double dxt = vx * dt;
	DBG("x change: " << dxt);
	double dyt = vy * dt;
	DBG("y change: " << dyt);
	double changeOfAngleInTime = angularVelocity * dt;
	DBG("angle change: " << changeOfAngleInTime);

	{
		std::lock_guard<std::mutex> lock(g_robotLocationMutex);
		m_coordination.x += dxt;
		m_coordination.y += dyt;
		m_coordination.angle += changeOfAngleInTime;
	}
}

