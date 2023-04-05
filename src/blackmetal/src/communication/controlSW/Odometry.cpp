#include "Odometry.hpp"

#include "ReturnStatus.hpp"
#include "controlSW/BlackMetal.hpp"
#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"
#include "types/RobotResponseType.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <ratio>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <variant>

/// Defined by the BlackMetal source code.
#define MAX_WHEEL_SPEED 0.8
/// The filter must be strong enough to filter out the noise.
#define FILTER_COEFFICIENT 0.8

std::mutex g_odometryMutex;
std::mutex g_robotLocationMutex;
constexpr std::chrono::milliseconds g_pollTime(100);

using namespace std::placeholders;

INIT_MODULE(Odometry, dbg_level::DBG);

Odometry::Odometry(std::shared_ptr<RobotDataDelegator> &robotDataDelegator)
	: m_robotDataDelegator(robotDataDelegator)
	, m_positionPublisher(nullptr)
	, m_coordination()
	, m_chassisLength(std::numeric_limits<double>::max())
	, m_wheelRadius(0)
	, m_leftWheelImpulseFilter(new RobotImpulseFilter(FILTER_COEFFICIENT))
	, m_rightWheelImpulseFilter(new RobotImpulseFilter(FILTER_COEFFICIENT))
{
	m_robotDataDelegator->setOnVelocityChangeCallback([this] (RobotRequestType newValue) {
		INFO("Resetting the filter values to " << newValue.toJson() << " because of a change request");
		m_leftWheelImpulseFilter->resetInitState(std::get<double>(newValue.leftWheel()));
		m_rightWheelImpulseFilter->resetInitState(std::get<double>(newValue.rightWheel()));
	});

	std::thread(
		[this] {
			while (m_robotDataDelegator->connected()) {
				std::chrono::duration<double, std::micro> microseconds{ Stopwatch::lastStoppedTime() };
				auto time = ((g_pollTime - microseconds) <= 0ms ? 0ms : g_pollTime - microseconds);

				if (time > 0ms) {
					DBG("Sleeping for " << time.count()/1'000'000. << " seconds");
					std::this_thread::sleep_for(time);
				}
				else {
					WARN("The loop time was longer than expected " << (-1*time).count() << "ms");
				}
				execute();
			}
		}
	).detach();
	INFO("Timer initialized");
}

Odometry::~Odometry()
{
	auto v = Stopwatch::getStoppedTimes();
	std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, ", "));
}

void Odometry::execute()
{
	static Speed lastValue;
	Speed wheels;

	TIC;

	m_robotDataDelegator->sendRequest(bm::Command::GET_LR_WHEEL_VELOCITY);
	RobotResponseType wheelImpulses = m_robotDataDelegator->robotVelocity();
	INFO("Before filtering " << wheelImpulses.toJson());

	// We must filter the impulses and not the mps, because the filter mus be reset to desired value on change request.
	// This is possible only with impulses.
	wheelImpulses.setLeftWheel(m_leftWheelImpulseFilter->filter(wheelImpulses.leftWheel()));
	wheelImpulses.setRightWheel(m_rightWheelImpulseFilter->filter(wheelImpulses.rightWheel()));
	INFO("After filtering " << wheelImpulses.toJson());

	// The impulses are taken from the struct and run thourgh low pass filter.
	// WARN: The rith wheel impulses are inverted.
	wheels = transformToVelocity(std::move(wheelImpulses));

	if (wheels.leftWheel != lastValue.leftWheel || wheels.rightWheel != lastValue.rightWheel) {
		INFO("Obtained speeds are " << wheels.leftWheel << " and " << wheels.rightWheel);
		lastValue = wheels;
	}

	{
		std::lock_guard lock(g_odometryMutex);
		m_velocity = wheels;
	}

	TOC;

	std::thread(std::bind(&Odometry::changeRobotLocation, this, _1), std::move(wheels)).detach();
}

double Odometry::leftWheelSpeed() const
{
	std::lock_guard lock(g_odometryMutex);
	return m_velocity.leftWheel;
}

double Odometry::rightWheelSpeed() const
{
	std::lock_guard lock(g_odometryMutex);
	return m_velocity.rightWheel;
}

void Odometry::setChassisLength(double chassisLength)
{
	std::lock_guard lock(g_odometryMutex);
	m_chassisLength = chassisLength;
}

void Odometry::setWheelRadius(double wheelRadius)
{
	std::lock_guard lock(g_odometryMutex);
	m_wheelRadius = wheelRadius;
}

double Odometry::getChassisLength()
{
	std::lock_guard lock(g_odometryMutex);
	return m_chassisLength;
}

double Odometry::getWheelRadius()
{
	std::lock_guard lock(g_odometryMutex);
	return m_wheelRadius;
}

void Odometry::setPositinoPublisher(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr positionPublisher)
{
	this->m_positionPublisher = positionPublisher;
}

Odometry::Speed Odometry::transformToVelocity(RobotResponseType &&response)
{
	double lws = response.leftWheel();
	double rws = response.rightWheel();

	lws = std::clamp(lws, 0., MAX_WHEEL_SPEED);
	rws = std::clamp(rws, 0., MAX_WHEEL_SPEED);

	return {lws, rws};
}

void Odometry::changeRobotLocation(Speed &&speed)
{
	speed = {
		.leftWheel = speed.leftWheel * g_pollTime.count() / 1'000,
		.rightWheel = speed.rightWheel * g_pollTime.count() / 1'000,
	};
	WARN("Traveled impulses lw: " << speed.leftWheel << " rw: " << speed.rightWheel);

	// Calculate the distance traveled by each wheel in meters
	double leftWheelDistance = 2.0 * M_PI * m_wheelRadius * speed.leftWheel / 1024.0;
	double rightWheelDistance = 2.0 * M_PI * m_wheelRadius * speed.rightWheel / 1024.0;
	WARN("lwDistance: " << leftWheelDistance << " rwDistance: " << rightWheelDistance);

	// Calculate the robot's linear and angular displacement in meters and radians, respectively
	double linearDisplacement = (leftWheelDistance + rightWheelDistance) / 2.0;
	double angularDisplacement = (rightWheelDistance - leftWheelDistance) / m_chassisLength;
	WARN("linear change: " << linearDisplacement << " angular change: " << angularDisplacement);

	// Calculate the robot's new position based on its current position and displacement
	{
		std::lock_guard lock(g_odometryMutex);
		// Replace with the robot's current x coordinate
		m_coordination.pose.pose.position.x += linearDisplacement * std::cos(angularDisplacement);
		// Replace with the robot's current y coordinate
		m_coordination.pose.pose.position.y += linearDisplacement * std::sin(angularDisplacement);
		// Replace with the robot's current heading in radians
		m_coordination.pose.pose.orientation.z = wrapAngle(m_coordination.pose.pose.orientation.z + angularDisplacement);

		if (m_positionPublisher) {
			m_positionPublisher->publish(m_coordination);
		}
	}
}

double Odometry::wrapAngle(double angle)
{
	// Calculate the reminder of deviding two doubles.
	angle = fmod(angle, 2.0 * M_PI);
	if (angle < 0.0) {
		angle += 2.0 * M_PI;
	}
	return angle;
}

