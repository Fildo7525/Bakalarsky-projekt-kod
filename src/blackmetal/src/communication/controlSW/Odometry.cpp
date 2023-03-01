#include "Odometry.hpp"

#include "ReturnStatus.hpp"
#include "controlSW/BlackMetal.hpp"
#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"
#include "types/RobotResponseType.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <ratio>
#include <stdexcept>
#include <thread>
#include <variant>

/// Defined by the BlackMetal source code.
#define MAX_WHEEL_SPEED 0.8
/// Constatns used to convert the impulses to meters per second.
/// They were obtained by tests and than by applying the linear regression to the measured samples.
#define FROM_IMP_TO_MPS_L 1261.13519092
#define FROM_IMP_TO_MPS_R 1188.08952528
/// The filter must be strong enough to filter out the noise.
#define FILTER_COEFFICIENT 0.8

std::mutex g_odometryMutex;
std::mutex g_robotLocationMutex;
constexpr std::chrono::milliseconds g_pollTime(250);

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
	m_robotDataDelegator->setOnVelocityChangeCallback([this] (RobotResponseType newValue) {
		INFO("Resetting the filter values to " << newValue.toJson() << " because of a change request");
		m_leftWheelImpulseFilter->resetInitState(newValue.leftWheel());
		m_rightWheelImpulseFilter->resetInitState(newValue.rightWheel());
	});

	std::thread(
		[this] {
			while (m_robotDataDelegator->connected()) {
				std::chrono::duration<double, std::micro> microseconds{ Stopwatch::lastStoppedTime() };
				auto time = ((g_pollTime - microseconds) <= 0ms ? 0ms : g_pollTime - microseconds);

				if (time > 0ms) {
					DBG("Sleeping for " << time.count()/1'000. << " seconds");
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
}

void Odometry::execute()
{
	static Speed lastValue;
	Speed wheels;

	TIC;

	m_robotDataDelegator->sendRequest(bm::Command::GET_LR_WHEEL_VELOCITY);
	RobotResponseType wheelImpulses = m_robotDataDelegator->robotVelocity();
	INFO("Before filtering " << wheelImpulses.toJson());

	// We must filter the impulses and not the mps, because the filter mus be reset to desired walue on chage request.
	// This is possible only with impulses.
	wheelImpulses.setLeftWheel(m_leftWheelImpulseFilter->filter(wheelImpulses.leftWheel()));
	wheelImpulses.setRightWheel(m_rightWheelImpulseFilter->filter(wheelImpulses.rightWheel()));
	INFO("After filtering " << wheelImpulses.toJson());

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

	double elapsedTime = Stopwatch::lastStoppedTime();
	std::thread(std::bind(&Odometry::changeRobotLocation, this, _1, _2), std::move(wheels), std::move(elapsedTime)).detach();
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

void Odometry::setPositinoPublisher(rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr positionPublisher)
{
	this->m_positionPublisher = positionPublisher;
}

Odometry::Speed Odometry::transformToVelocity(RobotResponseType &&response)
{
	double lws = response.leftWheel();
	double rws = response.rightWheel();

	// We need to convert the impulses send by robot to SI units (meters per second).
	lws = lws / FROM_IMP_TO_MPS_L;
	rws = rws / FROM_IMP_TO_MPS_R;

	lws = lws > MAX_WHEEL_SPEED ? 0 : lws;
	rws = rws > MAX_WHEEL_SPEED ? 0 : rws;

	return {lws, -rws};
}

void Odometry::changeRobotLocation(Speed &&speed, long double &&elapsedTime)
{
	// The poll time is in milliseconds while the elapsedTime is in microseconds.
	long double dt = (g_pollTime.count() + elapsedTime/1'000.) / 1'000.;
	DBG("dt: " << dt);
	double angularVelocity = (double(speed.rightWheel) - speed.leftWheel) / m_chassisLength;
	DBG("Angular velocity: " << angularVelocity);
	double speedInCenterOfGravity = (speed.rightWheel + speed.leftWheel) / 2.0;
	DBG("Centre of gravity velocity: " << speedInCenterOfGravity);
	// auto icr = m_chassisLength / 2.0 * (speed.rightWheel() + speed.leftWheel()) / (speed.rightWheel() - speed.leftWheel());

	double vx;
	double vy;
	{
		std::lock_guard lock(g_robotLocationMutex);
		vx = speedInCenterOfGravity * std::cos(m_coordination.z);
		vy = speedInCenterOfGravity * std::sin(m_coordination.z);
	}

	double dxt = vx * dt;
	DBG("x change: " << dxt);
	double dyt = vy * dt;
	DBG("y change: " << dyt);
	double changeOfAngleInTime = angularVelocity * dt;
	DBG("angle change: " << changeOfAngleInTime);

	{
		std::lock_guard lock(g_robotLocationMutex);
		m_coordination.x += dxt;
		SUCCESS("X: " << m_coordination.x);
		m_coordination.y += dyt;
		SUCCESS("Y: " << m_coordination.y);
		m_coordination.z += changeOfAngleInTime;
		SUCCESS("Angle: " << m_coordination.z);
		if (m_positionPublisher) {
			m_positionPublisher->publish(m_coordination);
		}
	}
}

