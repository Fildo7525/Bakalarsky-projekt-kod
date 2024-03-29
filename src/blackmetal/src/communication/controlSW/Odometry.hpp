#pragma once

#include "RobotDataDelegator.hpp"
#include "filter/RobotImpulseFilter.hpp"
#include "types/RobotResponseType.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

/**
 * @class Odometry
 * @brief Manages the received data containing left and right wheel speed using the control server.
 *
 * With a set frequency sends requests to server and evaluates the position, of where it is located
 * in the Cartesian plain. The beginning position after turning on 
 * the robot is [0, 0, 0]. The coordinates are represented as [x, y, quaternion].
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
	 * Therefore the object should call the execute function in different thread.
	 *
	 * @param controlClient Reference to the control software connected to the robot.
	 */
	explicit Odometry(std::shared_ptr<RobotDataDelegator> robotDataDelegator);

	/// Destructor.
	~Odometry();

	/**
	 * @brief Function polling the robot for its left and right wheel speed.
	 * WARN: The function WILL block the thread, thus it must be run in different thread.
	 */
	void execute();

	/**
	 * @brief Sets the length of the robot chassis.
	 *
	 * @param chassisLength Value to be set.
	 */
	void setChassisLength(double chassisLength);

	/**
	 * @brief Sets the radius of left and right wheel speed.
	 *
	 * @param wheelRadius Value to be set.
	 */
	void setWheelRadius(double wheelRadius);

	/**
	 * @brief Sets the encoder resolution.
	 *
	 * @param encoderResolution Value to be set.
	 */
	void setEncoderResolution(int encoderResolution);

	/**
	 * @brief Returns the length of the chassis.
	 */
	double chassisLength() const;

	/**
	 * @brief Returns the radius of the wheels.
	 */
	double wheelRadius() const;

	/**
	 * @brief Sets the position publisher created by @c rclcpp::Node
	 *
	 * @param positionPublisher Publisher of the odometry messages.
	 */
	void setPositinoPublisher(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr positionPublisher);

protected:
	/**
	 * @brief Retrieve the left and right wheel speed from the received json message.
	 *
	 * @see m_leftWheelImpulseFilter Low pass filter filtering the impulses of the left wheel.
	 * @see m_rightWheelImpulseFilter Low pass filter filtering the impulses of the right wheel.
	 *
	 * @warn This method inverts the right wheel speed so that we could calculate the position of the robot.
	 *
	 * @param response Message received from the server.
	 * @return @c Speed structure of left and right wheel speed.
	 */
	Speed getImpulsesFromResponse(RobotResponseType &&response) const;

	/**
	 * @brief Changes the robot location based on the left and right wheel velocity.
	 *
	 * The location is calculated from the period of the polling execute function
	 * and the time that takes to obtain the velocities of the wheels.
	 *
	 * @see execute The polling function.
	 *
	 * @param speed Rvalue reference to a structure containing the left and right wheel velocity.
	 */
	void changeRobotLocation(Speed &&speed);

	/**
	 * @brief Computes the angle of the robot in interval [-pi, pi] on Cartesian plain.
	 *
	 * @param angle Angle to be wrapped in radians.
	 * @return Wrapped angle in radians.
	 */
	double wrapAngle(double angle) const;

private:
	/// Instance of the control software client.
	std::shared_ptr<RobotDataDelegator> m_robotDataDelegator;

	std::thread m_workerThread;

	/// Publisher of the current robot position in robots Cartesian system.
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_positionPublisher;

	/// Timer invoking the execute function.
	rclcpp::TimerBase::SharedPtr m_timer;

	/// The robot coordinates in system where its initial position is [0, 0, 0] => (x, y, angle).
	nav_msgs::msg::Odometry m_coordination;

	/// The length of the robot chassis.
	double m_chassisLength;

	/// The left and right wheel radius.
	double m_wheelRadius;

	/// How many impulses will the encoder send per one turn.
	int m_encoderResolution;

	/// The filter of the robot's left motor impulses.
	std::shared_ptr<FrequencyFilter> m_leftWheelImpulseFilter;

	/// The filter of the robot's right motor impulses.
	std::shared_ptr<FrequencyFilter> m_rightWheelImpulseFilter;

	/// The last time the execute function was called.
	std::chrono::system_clock::time_point m_lastTime;

	/// The rotation of the robot around the z axis.
	long double m_orientationZ;
};

