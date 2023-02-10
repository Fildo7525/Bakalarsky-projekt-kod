#pragma once

#include <memory>
#include <thread>
#include "Client.hpp"

#include <rclcpp/timer.hpp>

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
	 * @class Coord
	 * @brief Specifies the coordinates and angle of rotation of the robot.
	 */
	struct Coord
	{
		/// The x coordinate of the robot in Cartesian plain.
		double x;

		/// The y coordinate of the robot in Cartesian plain.
		double y;

		/// The angle of the robot with the x axis in Cartesian plain.
		double angle;
	};

	/**
	 * @class Speed
	 * @brief Stores the speeds of the left and right wheel obtained from the robot.
	 */
	struct Speed
	{
		/// The velocity of left wheel.
		long leftWheel;

		/// The velocity of right wheel.
		long rightWheel;
	};

	/**
	 * @brief Creates an odometry object. This object polls the server for its velocity.
	 * Therefor the object should call the execute function in different thread.
	 *
	 * @param controlClient Reference to the control software connected to the robot.
	 */
	explicit Odometry(std::shared_ptr<Client> &controlClient);

	/**
	 * @brief Function polling the robot for its left and right wheel speed.
	 * The function will block the thread, thus it should be run in different thread.
	 */
	void execute();

	/**
	 * @brief Retrieve the left and right wheel speed from the received json message.
	 *
	 * @param jsonMessage Message received from the server.
	 * @return Structure of left and right wheel speed.
	 */
	Speed obtainWheelSpeeds(std::string &&jsonMessage) const;

	/**
	 * @brief Get the last left wheel speed.
	 *
	 * The output of this function is copy of the actual value.
	 * Thus you can use this variable in non thread safe functions.
	 */
	long leftWheelSpeed() const;

	/**
	 * @brief Get the last right wheel speed.
	 *
	 * The output of this function is copy of the actual value.
	 * Thus you can use this variable in non thread safe functions.
	 */
	long rightWheelSpeed() const;

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
private:

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
	std::shared_ptr<Client> m_controlClient;

	/// Timer invoking the execute function.
	rclcpp::TimerBase::SharedPtr m_timer;

	/// The robot coordinates in system where its initial position is [0, 0, 0] => (x, y, angle).
	Coord m_coordination;

	/// Thread that polls the server at a certain frequency for the wheels velocity.
	std::thread m_robotSpeedReceiver;

	// The speeds in the blackmetal code are defined as longs.
	Speed m_velocity;

	/// The length of the robot chassis.
	double m_chassisLength;

	/// The left and right wheel radius.
	double m_wheelRadius;
};

