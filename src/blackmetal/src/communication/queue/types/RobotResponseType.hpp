#pragma once

#include <string>

/**
 * @class RobotResponseType
 * @brief Json response from the robot.
 *
 * This class represents the json response from the robot. There is a possibility
 * to set the left and right wheel speed. The class also provides a method for
 * converting the object to json and vice versa.
 */
class RobotResponseType
{
public:
	/// Default constructor.
	RobotResponseType() = default;

	/**
	 * @brief Sets the left wheel speed in the json response representation.
	 *
	 * @param leftWheel Speed to be set in the json.
	 * @return Reference to the object.
	 */
	RobotResponseType& setLeftWheel(long leftWheel);

	/**
	 * @brief Sets the right wheel speed in the json response representation.
	 *
	 * @param rightWheel Speed to be set in the json.
	 * @return Reference to the object.
	 */
	RobotResponseType& setRightWheel(long rightWheel);

	/**
	 * @brief Get the left wheel speed.
	 */
	long leftWheel() const;

	/**
	 * @brief Get the right wheel speed.
	 */
	long rightWheel() const;

	/**
	 * @brief Returns the representation of the class in json format.
	 */
	std::string toJson() const;

	/**
	 * @brief Creates the object from the json representation.
	 *
	 * The function parses the json and creates the object from it.
	 *
	 * @param json Json representation of the object.
	 * @return Object created from the json.
	 */
	static RobotResponseType fromJson(const std::string &json);

	/**
	 * @brief Compares the left and right wheel speed.
	 *
	 * @param other Object to be compared with.
	 * @return Alwais false
	 */
	bool operator>(const RobotResponseType &other) const;

private:
	/// Left wheel speed.
	double m_leftWheel;

	/// Right wheel speed.
	double m_rightWheel;
};

