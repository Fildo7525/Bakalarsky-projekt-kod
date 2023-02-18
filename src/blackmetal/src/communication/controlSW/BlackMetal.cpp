#include "BlackMetal.hpp"

#include "colors.hpp"
#include "log.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#define PORT 665

INIT_MODULE(BlackMetal, dbg_level::DBG);

BlackMetal::BlackMetal()
	: rclcpp::Node("blackmetal")
	, m_robotDataReceiver(new RobotDataReceiver(PORT, "192.168.1.3"))
	, m_odometry(new Odometry(m_robotDataReceiver))
{
	m_odometry->setChassisLength(declare_parameter<double>("chasis", 1));
	m_odometry->setWheelRadius(declare_parameter<double>("wheelRadius", 0.2));

	DBG("Chasis has lenght " << m_odometry->getChassisLength() << " m");
	DBG("Wheel has radius " << m_odometry->getWheelRadius() << " m");
	DBG("Address set to " << m_robotDataReceiver->address() << ":" << PORT);

	m_twistSubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel",
		1,
		[this] (const geometry_msgs::msg::Twist &msg) {
			this->onTwistRecievedSendJson(msg);
		}
	);

	m_positionPublisher = this->create_publisher<geometry_msgs::msg::Vector3>("position", 10);
	m_odometry->setPositinoPublisher(m_positionPublisher);

	WARN("Odometry created");
}

void BlackMetal::onTwistRecievedSendJson(const geometry_msgs::msg::Twist &msg)
{
	static constexpr double toMps = 10;
	// The maximal linear velocity is 0.8 m/s and the maximal angle velocity is 2.857 rad/s.
	DBG("Message geometry_msgs::msg::Twist: " << geometry_msgs::msg::to_yaml(msg));
	m_rightWheelSpeed = (msg.linear.x + 0.5 * m_odometry->getChassisLength() * msg.angular.z)/ m_odometry->getWheelRadius() / toMps;
	m_leftWheelSpeed = (msg.linear.x - 0.5 * m_odometry->getChassisLength() * msg.angular.z)/ m_odometry->getWheelRadius() / toMps;
	INFO("Right wheel speed: " << m_rightWheelSpeed << " Left wheel speed: " << m_leftWheelSpeed);

	std::clamp(m_rightWheelSpeed, -1., 1.);
	std::clamp(m_leftWheelSpeed, -1., 1.);

	// WARN: When we change the robot velocity the filter will enlarge the transition time to the requested velocity.
	// In this case we have to forcefully reset the state of the filter. This should probably happen in RobotDataReceiver
	// so that we do not analyze old samples in the already reset filter.
	m_robotDataReceiver->requestSpeed(m_rightWheelSpeed, m_leftWheelSpeed);
}

const double& BlackMetal::chassisLength()
{
	return m_odometry->getChassisLength();
}

const double& BlackMetal::wheelRadius()
{
	return m_odometry->getWheelRadius();
}

