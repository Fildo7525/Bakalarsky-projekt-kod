#include "Odometry.hpp"

#include "ReturnStatus.hpp"
#include "controlSW/BlackMetal.hpp"
#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <variant>

/// Defined by the BlackMetal source code.
#define MAX_WHEEL_SPEED 1000

std::mutex g_odometryMutex;
std::mutex g_robotLocationMutex;
// WARN: The 3 second interval is just for debugging
constexpr std::chrono::milliseconds g_pollTime(3000);

using namespace std::placeholders;

INIT_MODULE(Odometry, dbg_level::DBG);

Odometry::Odometry(std::shared_ptr<Client> &controlClient)
	: m_controlClient(controlClient)
	, m_coordination({0, 0, 0})
	, m_chassisLength(std::numeric_limits<double>::max())
	, m_wheelRadius(0)
{
	m_robotSpeedReceiver = std::thread(
		[this] {
			while (m_controlClient->connected()) {
				std::chrono::duration<double, std::milli> milliseconds{ Stopwatch::lastStoppedTime() };
				auto time = ((g_pollTime - milliseconds) <= 0ms ? 0ms : g_pollTime - milliseconds);

				if (time > 0ms) {
					DBG("Sleeping for " << time.count());
					std::this_thread::sleep_for(time);
				}
				else {
					WARN("The loop time was longer than expected " << (-1*time).count() << "ms");
				}
				execute();
			}
		}
	);
	INFO("Timer initialized");
}

void Odometry::execute()
{
	static Speed lastValue;
	Speed wheels;

	TIC;

	m_controlClient->sendRequest(bm::Command::GET_LR_WHEEL_VELOCITY);
	std::string wheelSpeed = m_controlClient->robotVelocity();
	wheels = obtainWheelSpeeds(std::move(wheelSpeed));

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

Odometry::Speed Odometry::obtainWheelSpeeds(std::string &&jsonMessage) const
{
	// The structure will arrive in a wannabe json format
	// {"LeftWheelSpeed"=%ld "RightWheelSpeed"=%ld}
	// TODO: check if the try-catch block is necessary
	long lws, rws;
	try {
		DBG("Next attempt: " << jsonMessage);
		auto lws_start = jsonMessage.find_first_of(':') + 1;
		auto lws_end = jsonMessage.find_first_of(' ');
		lws = std::stol(jsonMessage.substr(lws_start, lws_end));

		auto rws_start = jsonMessage.find_last_of(':') + 1;
		auto rws_end = jsonMessage.find_last_of('}');
		rws = std::stol(jsonMessage.substr(rws_start, rws_end));
	} catch (std::out_of_range &e) {
		DBG("Attempting to receive the json on next read, current: " << std::quoted(jsonMessage));
		std::string nextAttempt;
		nextAttempt = m_controlClient->robotVelocity();
		return obtainWheelSpeeds(std::move(nextAttempt));
	} catch (std::exception &e) {
		ERR(e.what() << " with string " << std::quoted(jsonMessage));
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

void Odometry::setChassisLength(double chassisLength)
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	m_chassisLength = chassisLength;
}

void Odometry::setWheelRadius(double wheelRadius)
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	m_wheelRadius = wheelRadius;
}

const double &Odometry::getChassisLength()
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	return m_chassisLength;
}

const double &Odometry::getWheelRadius()
{
	std::lock_guard<std::mutex> lock(g_odometryMutex);
	return m_wheelRadius;
}

void Odometry::changeRobotLocation(Speed &&speed, long double &&elapsedTime)
{
	// The poll time is in milliseconds while th elapsedTime is in microseconds.
	long double dt = (g_pollTime.count() + elapsedTime/1'000.) / 1'000.;
	DBG("dt: " << dt);
	double angularVelocity = (speed.rightWheel - speed.leftWheel) / m_chassisLength;
	DBG("Angular velocity: " << angularVelocity);
	double speedInCenterOfGravity = (speed.rightWheel + speed.leftWheel) / 2.0;
	DBG("CoG velocity: " << speedInCenterOfGravity);
	// auto icr = m_controlClient->chassisLength() / 2.0 * (speed.rightWheel + speed.leftWheel) / (speed.rightWheel - speed.leftWheel);

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

