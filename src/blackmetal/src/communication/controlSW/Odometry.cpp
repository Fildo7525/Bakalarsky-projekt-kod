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

#include <tf2/LinearMath/Quaternion.h>

/// Defined by the BlackMetal source code.
#define MAX_WHEEL_SPEED 0.8
/// The filter must be strong enough to filter out the noise.
#define FILTER_COEFFICIENT 0.8

std::mutex g_odometryMutex;
std::mutex g_robotLocationMutex;
using namespace std::chrono_literals;
constexpr std::chrono::milliseconds g_pollTime(100);

using namespace std::placeholders;

INIT_MODULE(Odometry, Logger::level::DBG);

Odometry::Odometry(std::shared_ptr<RobotDataDelegator> robotDataDelegator)
	: m_robotDataDelegator(robotDataDelegator)
	, m_positionPublisher(nullptr)
	, m_coordination()
	, m_chassisLength(std::numeric_limits<double>::max())
	, m_wheelRadius(0)
	, m_leftWheelImpulseFilter(new RobotImpulseFilter(FILTER_COEFFICIENT))
	, m_rightWheelImpulseFilter(new RobotImpulseFilter(FILTER_COEFFICIENT))
{
	m_robotDataDelegator->setOnVelocityChangeCallback([this] (RobotRequestType newValue) {
		if (newValue.command() == bm::Command::SET_LR_WHEEL_VELOCITY
			|| newValue.command() == bm::Command::SET_LR_WHEEL_POSITION) {
			INFO("Resetting the filter values to " << std::quoted(newValue.toJson()) << " because of a change request");
			m_leftWheelImpulseFilter->resetInitState(std::get<double>(newValue.leftWheel()));
			m_rightWheelImpulseFilter->resetInitState(std::get<double>(newValue.rightWheel()));
		}
		else {
			WARN("Filters cannot be reset with json " << std::quoted(newValue.toJson()));
		}
	});

	m_orientationZ = 0;

	std::thread(
		[this] {
			while (m_robotDataDelegator->connected()) {
				m_lastTime = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double, std::micro> microseconds{ Stopwatch::lastStoppedTime() };
				std::chrono::duration<long double, std::micro> time = ((g_pollTime - microseconds) <= 0ms ? 0ms : g_pollTime - microseconds);

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
	);
	INFO("Timer initialized");
}

Odometry::~Odometry()
{
	auto v = Stopwatch::getStoppedTimes();
	std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, ", "));
}

void Odometry::execute()
{
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

	// The impulses are taken from the struct and run through low pass filter.
	// WARN: The right wheel impulses are inverted.
	wheels = getImpulsesFromResponse(std::move(wheelImpulses));

	INFO("Obtained speeds are " << wheels.leftWheel << " and " << wheels.rightWheel);

	TOC;

	std::thread(std::bind(&Odometry::changeRobotLocation, this, _1), std::move(wheels)).detach();
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

void Odometry::setEncoderResolution(int encoderResolution)
{
	std::lock_guard lock(g_odometryMutex);
	m_encoderResolution = encoderResolution;
}

double Odometry::chassisLength() const
{
	std::lock_guard lock(g_odometryMutex);
	return m_chassisLength;
}

double Odometry::wheelRadius() const
{
	std::lock_guard lock(g_odometryMutex);
	return m_wheelRadius;
}

void Odometry::setPositinoPublisher(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr positionPublisher)
{
	this->m_positionPublisher = positionPublisher;
}

Odometry::Speed Odometry::getImpulsesFromResponse(RobotResponseType &&response) const
{
	double lws = response.leftWheel();
	double rws = response.rightWheel();

	return {lws, -rws};
}

void Odometry::changeRobotLocation(Speed &&speed)
{
	// Calculate the robot's new position based on its current position and displacement
	DBG("Traveled impulses lw: " << speed.leftWheel << " rw: " << speed.rightWheel);
	// Calculate the distance traveled by each wheel in meters
	double leftWheelFloorVelocity = 2.0 * M_PI * m_wheelRadius * speed.leftWheel / m_encoderResolution;
	double rightWheelFloorVelocity = 2.0 * M_PI * m_wheelRadius * speed.rightWheel / m_encoderResolution;
	DBG("lwDistance: " << leftWheelFloorVelocity << " rwDistance: " << rightWheelFloorVelocity);

	// Calculate the robot's linear and angular displacement in meters and radians, respectively
	double linearVelocity = (leftWheelFloorVelocity + rightWheelFloorVelocity) / 2.0;
	double angularVelocity = (rightWheelFloorVelocity - leftWheelFloorVelocity) / m_chassisLength;
	DBG("linear change: " << linearVelocity << " angular change: " << angularVelocity);
	{
		std::lock_guard lock(g_odometryMutex);
		auto now = std::chrono::high_resolution_clock::now();
		auto tmp = now - m_lastTime;
		m_lastTime = now;
		double dt = std::chrono::duration_cast<std::chrono::microseconds>(tmp).count() / 1'000.;
		DBG("dt: " << dt);

		m_orientationZ = wrapAngle(m_orientationZ + angularVelocity * dt);
		auto poseX = linearVelocity * std::cos(m_orientationZ) * dt;
		auto poseY = linearVelocity * std::sin(m_orientationZ) * dt;

		tf2::Quaternion q;
		q.setRPY(0, 0, m_orientationZ);
		DBG("Quaternion: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w());

		// Set the linear velocity for the odometry message.
		m_coordination.header.stamp.sec = now.time_since_epoch().count();
		m_coordination.twist.twist.linear.x = linearVelocity;
		m_coordination.twist.twist.linear.y = 0.0;
		m_coordination.twist.twist.linear.z = 0.0;

		// Set the angular velocity for the odometry message.
		m_coordination.twist.twist.angular.x = 0.0;
		m_coordination.twist.twist.angular.y = 0.0;
		m_coordination.twist.twist.angular.z = angularVelocity;

		// Replace with the robot's current coordinates
		m_coordination.pose.pose.position.x += poseX;
		m_coordination.pose.pose.position.y += poseY;
		m_coordination.pose.pose.position.z = 0.0;

		// Replace with the robot's current heading in radians
		m_coordination.pose.pose.orientation.x = q.x();
		m_coordination.pose.pose.orientation.y = q.y();
		m_coordination.pose.pose.orientation.z = q.z();
		m_coordination.pose.pose.orientation.w = q.w();

		if (m_positionPublisher) {
			m_positionPublisher->publish(m_coordination);
		}
	}
}

double Odometry::wrapAngle(double angle) const
{
	// Calculate the reminder of deviding two doubles.
	angle = fmod(angle, 2.0 * M_PI);
	if (angle < 0.0) {
		angle += 2.0 * M_PI;
	}
	INFO("The angle was wrapped to: " << angle);
	return angle;
}

