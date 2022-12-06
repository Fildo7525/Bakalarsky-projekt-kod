#include "BlackMetal.hpp"

#include "colors.hpp"
#include "log.hpp"

#include <chrono>
#include <functional>

#define PORT 665

INIT_MODULE(BlackMetal, dbg_level::DBG);

BlackMetal::BlackMetal()
	: rclcpp::Node("blackmetal")
	, m_controlClient(new Client(PORT, "192.168.1.3"))
	, m_odometry(new Odometry(m_controlClient))
{
	m_odometry->setChassisLength(declare_parameter<double>("chasis", 1));
	m_odometry->setWheelRadius(declare_parameter<double>("wheelRadius", 0.2));

	DBG("Chasis has lenght " << m_odometry->getChassisLength() << " m");
	DBG("Wheel has radius " << m_odometry->getWheelRadius() << " m");
	DBG("Address set to " << m_controlClient->address() << ":" << PORT);

	m_twistSubscriber
		= this->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel",
			1,
			[this] (const geometry_msgs::msg::Twist &msg) {
				this->onTwistRecievedSendJson(msg);
			}
	);
	WARN("Odometry created");
}

void BlackMetal::onTwistRecievedSendJson(const geometry_msgs::msg::Twist &msg)
{
	DBG("Message geometry_msgs::msg::Twist: " << geometry_msgs::msg::to_yaml(msg));
	m_rightWheelSpeed = (msg.linear.x + 0.5 * m_odometry->getChassisLength() * msg.angular.z)/ m_odometry->getWheelRadius() / 1000.;
	m_leftWheelSpeed = (msg.linear.x - 0.5 * m_odometry->getChassisLength() * msg.angular.z)/ m_odometry->getWheelRadius() / 1000.;
	INFO("Right wheel speed: " << m_rightWheelSpeed << " Left wheel speed: " << m_leftWheelSpeed);

	if (m_rightWheelSpeed > 1) {
		m_rightWheelSpeed = 1;
	} else if (m_rightWheelSpeed < -1) {
		m_rightWheelSpeed = -1;
	}
	if (m_leftWheelSpeed > 1) {
		m_leftWheelSpeed = 1;
	} else if (m_leftWheelSpeed < -1) {
		m_leftWheelSpeed = -1;
	}

	m_controlClient->requestSpeed(m_rightWheelSpeed, m_leftWheelSpeed);
}

const double& BlackMetal::chassisLength()
{
	return m_odometry->getChassisLength();
}

const double& BlackMetal::wheelRadius()
{
	return m_odometry->getWheelRadius();
}

