#pragma once

class Odometry;
#include "controlSW/BlackMetal.hpp"

#include <rclcpp/timer.hpp>

/**
 * @class Odometry
 * @brief Manages left and right wheel speed using the control server.
 *
 * With a set frequency sends requests to server and evaluates the position, of where it is located
 * in the Cartesian plain. The beginning position after turning on 
 * the robot is [0, 0, 0]. The coordinates are represented as [x, y, angle].
 *
 */
class Odometry
{
public:
	/**
	 * @class Coord
	 * @brief Specifies the coordinates and angle of rotation of the robot.
	 */
	struct Coord {
		/// The x coordinate of the robot in Cartesian plain.
		int x;
		/// The y coordinate of the robot in Cartesian plain.
		int y;
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
	 * @param controlSoftware Reference to the control software connected to the robot.
	 */
	explicit Odometry(BlackMetal &controlSoftware);

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
	Speed obtainWheelSpeeds(const std::string &jsonMessage);

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
private:
	/**
	 * @brief Evaluates the received message.
	 *
	 * @param returnJson Json containing whether the server could parse the received string.
	 */
	bm::Status evalReturnState(const std::string &returnJson);

	/**
	 * @brief Changes the robot location based on the left and right wheel velocity.
	 *
	 * The location is calculated from the period of the polling execute function and the
	 * time that takes to obtain the velocities of the wheels.
	 * @see execute the polling function.
	 *
	 * @param speed Structure containing the left and right wheel velocity.
	 */
	void changeRobotLocation(Speed &&speed);

private:
	/// Instance of the control software client.
	BlackMetal &m_controlSoftware;
	/// Timer invoking the execute function.
	rclcpp::TimerBase::SharedPtr m_timer;
	/// The robot coordinates in system where its initial position is [0, 0, 0] => (x, y, angle).
	Coord m_coordination;

	// The speeds in the blackmetal code are defined as longs.
	Speed m_velocity;
	double m_lastMeasuredTime;
};
