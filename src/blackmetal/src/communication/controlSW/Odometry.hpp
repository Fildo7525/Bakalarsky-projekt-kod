#pragma once

#include "RobotDataReceiver.hpp"
#include "controlSW/filter/RobotImpulseFilter.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

#include <memory>
#include <thread>

/**
 * @class Odometry
 * @brief Manages left and right wheel speed using the control server.
 *
 * With a set frequency sends requests to server and evaluates the position, of where it is located
 * in the Cartesian plain. The beginning position after turning on 
 * the robot is [0, 0, 0]. The coordinates are represented as [x, y, angle].
 */
class Odometry
{
public:
	/**
	 * @class Speed
	 * @brief Stores the speeds of the left and right wheel obtained from the robot.
	 */
	struct Speed
	{
		/// The velocity of left wheel.
		double leftWheel;

		/// The velocity of right wheel.
		double rightWheel;
	};

	/**
	 * @brief Creates an odometry object. This object polls the server for its velocity.
	 * Therefor the object should call the execute function in different thread.
	 *
	 * @param controlClient Reference to the control software connected to the robot.
	 */
	explicit Odometry(std::shared_ptr<RobotDataReceiver> &robotDataReceiver);

	~Odometry();

	/**
	 * @brief Function polling the robot for its left and right wheel speed.
	 * The function will block the thread, thus it should be run in different thread.
	 */
	void execute();

	/**
	 * @brief Get the last left wheel speed.
	 *
	 * The output of this function is copy of the actual value.
	 * Thus you can use this variable in non thread safe functions.
	 */
	[[maybe_unused]] long leftWheelSpeed() const;

	/**
	 * @brief Get the last right wheel speed.
	 *
	 * The output of this function is copy of the actual value.
	 * Thus you can use this variable in non thread safe functions.
	 */
	[[maybe_unused]] long rightWheelSpeed() const;

	/**
	 * @brief Sets the length of the robot chassis.
	 *
	 * @param chassisLength  value to be set.
	 */
	void setChassisLength(double chassisLength);

	/**
	 * @brief Sets the radius of left and right wheel speed.
	 *
	 * @param wheelRadius value to be set.
	 */
	void setWheelRadius(double wheelRadius);

	/**
	 * @brief Returns the lenth of the chassis.
	 */
	const double &getChassisLength();

	/**
	 * @brief Returns the radius of the wheels.
	 */
	const double &getWheelRadius();

	/**
	 * @brief Sets the position publisher created by rclcpp::Node
	 *
	 * @param positionPublisher 
	 */
	void setPositinoPublisher(rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr positionPublisher);

private:
	/**
	 * @brief Retrieve the left and right wheel speed from the received json message.
	 *
	 * This method transforms the left and right wheel impulses to metres per second.
	 * To ensure the best possible outcome the impulses are run through a low pass filter.
	 *
	 * @see m_leftWheelImpulseFilter Low pass filter filtering the impulses of the left wheel.
	 * @see m_rightWheelImpulseFilter Low pass filter filtering the impulses of the right wheel.
	 *
	 * @param jsonMessage Message received from the server.
	 * @return Structure of left and right wheel speed.
	 */
	Speed obtainWheelSpeeds(std::string &&jsonMessage);

	/**
	 * @brief Changes the robot location based on the left and right wheel velocity.
	 *
	 * The location is calculated from the period of the polling execute function and the
	 * time that takes to obtain the velocities of the wheels.
	 * @see execute the polling function.
	 *
	 * @param speed Rvalue reference to a structure containing the left and right wheel velocity.
	 * @param elapsedTime The rvalue reference to the elapsed time after the receiving the robot velocity. 
	 */
	void changeRobotLocation(Speed &&speed, long double &&elapsedTime);

private:
	/// Instance of the control software client.
	std::shared_ptr<RobotDataReceiver> m_robotDataReceiver;

	/// Publisher of the current robot position in robots Cartesian system.
	rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_positionPublisher;

	/// Timer invoking the execute function.
	rclcpp::TimerBase::SharedPtr m_timer;

	/// The robot coordinates in system where its initial position is [0, 0, 0] => (x, y, angle).
	geometry_msgs::msg::Vector3 m_coordination;

	/// Thread that polls the server at a certain frequency for the wheels velocity.
	std::thread m_robotWorkerThread;

	// The speeds in the blackmetal code are defined as longs.
	Speed m_velocity;

	/// The length of the robot chassis.
	double m_chassisLength;

	/// The left and right wheel radius.
	double m_wheelRadius;

	/// The filter of the robot's left motor impulses.
	RobotImpulseFilter m_leftWheelImpulseFilter;

	/// The filter of the robot's right motor impulses.
	RobotImpulseFilter m_rightWheelImpulseFilter;
};

