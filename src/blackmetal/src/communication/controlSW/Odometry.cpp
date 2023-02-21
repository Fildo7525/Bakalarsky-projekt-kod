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
#define MAX_WHEEL_SPEED 0.8
#define FROM_IMP_TO_MPS_L 1012.53636364
#define FROM_IMP_TO_MPS_R 1053.67588345
/// The filter must be strong enough to filter out the noise.
#define FILTER_COEFFICIENT 0.999

std::mutex g_odometryMutex;
std::mutex g_robotLocationMutex;
// WARN: The 1 second interval is just for debugging
constexpr std::chrono::milliseconds g_pollTime(1000);

using namespace std::placeholders;

INIT_MODULE(Odometry, dbg_level::DBG);

Odometry::Odometry(std::shared_ptr<RobotDataReceiver> &robotDataReceiver)
	: m_robotDataReceiver(robotDataReceiver)
	, m_positionPublisher(nullptr)
	, m_coordination()
	, m_chassisLength(std::numeric_limits<double>::max())
	, m_wheelRadius(0)
	, m_leftWheelImpulseFilter(FILTER_COEFFICIENT)
	, m_rightWheelImpulseFilter(FILTER_COEFFICIENT)
{
	m_robotDataReceiver->setOnVelocityChangeCallback([this] (RobotResponseType newValue) {
		m_leftWheelImpulseFilter.resetInitState(newValue.leftWheel());
		m_rightWheelImpulseFilter.resetInitState(newValue.rightWheel());
	});

	m_robotWorkerThread = std::thread(
		[this] {
			while (m_robotDataReceiver->connected()) {
				std::this_thread::sleep_for(g_pollTime);
				execute();
			}
		}
	);
	INFO("Timer initialized");
}

Odometry::~Odometry()
{
	m_robotWorkerThread.join();
}

void Odometry::execute()
{
	static RobotResponseType lastValue;
	RobotResponseType wheels;

	TIC;

	m_robotDataReceiver->sendRequest(bm::Command::GET_LR_WHEEL_VELOCITY);
	std::string wheelSpeed = m_robotDataReceiver->robotVelocity();
	INFO("Odometry has received a message from robot");
	wheels = obtainWheelSpeeds(std::move(wheelSpeed));

	if (wheels.leftWheel() != lastValue.leftWheel() || wheels.rightWheel() != lastValue.rightWheel()) {
		INFO("Obtained speeds are " << wheels.leftWheel() << " and " << wheels.rightWheel());
		lastValue = wheels;
	}

	{
		std::lock_guard lock(g_odometryMutex);
		m_velocity.setLeftWheel(wheels.leftWheel());
		m_velocity.setRightWheel(wheels.rightWheel());
	}

	TOC;

	double elapsedTime = Stopwatch::lastStoppedTime();
	std::thread(std::bind(&Odometry::changeRobotLocation, this, _1, _2), std::move(wheels), std::move(elapsedTime)).detach();
}

long Odometry::leftWheelSpeed() const
{
	std::lock_guard lock(g_odometryMutex);
	return m_velocity.leftWheel();
}

long Odometry::rightWheelSpeed() const
{
	std::lock_guard lock(g_odometryMutex);
	return m_velocity.rightWheel();
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

const double &Odometry::getChassisLength()
{
	std::lock_guard lock(g_odometryMutex);
	return m_chassisLength;
}

const double &Odometry::getWheelRadius()
{
	std::lock_guard lock(g_odometryMutex);
	return m_wheelRadius;
}

void Odometry::setPositinoPublisher(rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr positionPublisher)
{
	this->m_positionPublisher = positionPublisher;
}

RobotResponseType Odometry::obtainWheelSpeeds(std::string &&jsonMessage)
{
	// The structure will arrive in a wannabe json format
	// {"LeftWheelSpeed"=%ld "RightWheelSpeed"=%ld}

	RobotResponseType response = RobotResponseType::fromJson(jsonMessage);
	double lws = response.leftWheel();
	double rws = response.rightWheel();

	// We need to convert the impulses send by robot to SI units (meters per second).
	lws = lws / FROM_IMP_TO_MPS_L;
	rws = rws / FROM_IMP_TO_MPS_R;

	lws = lws > MAX_WHEEL_SPEED ? 0 : lws;
	rws = rws > MAX_WHEEL_SPEED ? 0 : rws;

	response.setLeftWheel(lws);
	response.setRightWheel(rws);

	return response;
}

void Odometry::changeRobotLocation(RobotResponseType &&speed, long double &&elapsedTime)
{
	// The poll time is in milliseconds while the elapsedTime is in microseconds.
	long double dt = (g_pollTime.count() + elapsedTime/1'000.) / 1'000.;
	DBG("dt: " << dt);
	double angularVelocity = (double(speed.rightWheel()) - speed.leftWheel()) / m_chassisLength;
	DBG("Angular velocity: " << angularVelocity);
	double speedInCenterOfGravity = (speed.rightWheel() + speed.leftWheel()) / 2.0;
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

