#pragma once

/**
 * @namespace bm
 * @brief namespace containing the enums for blackmetal constants.
 */
namespace bm
{

/**
 * @enum Command
 * @brief Commands defined in the blackmetal robot documentation.
 */
enum class Command {
	/// Empty command
	EMPTY,
	/// Emergency stop
	EMG_STOP,
	/// Normal stop
	NORMAL_STOP,
	/// Sets Left and right wheel velocity.
	/// The velocities are required for this command.
	SET_LR_WHEEL_VELOCITY,
	/// Set the position to which should the wheels turn to.
	SET_LR_WHEEL_POSITION,
	/// Not implemented.
	NONE_5,
	/// Get left and right wheel actual velocities.
	GET_LR_WHEEL_VELOCITY,
	/// Prepare the controller of left and right wheel.
	PREPARE_WHEEL_CONTROLLER,
	/// Get left and right wheel actual positions from robot.
	GET_LR_WHEEL_POSITION,
};

/**
 * @enum Status
 * @brief return status of the server.
 *
 * The robot receives the json string and returns a json string with a specific return status.
 * This status is than mapped on BlackMetal::Status enum type.
 */
enum class Status {
	/// Server processed the request.
	OK,
	/// Server could not process the request. The buffer is full.
	FULL_BUFFER,
	/// The client could not send data to the server.
	SEND_ERROR,
	/// The response could not be received.
	RECEIVE_ERROR,
};

} // namespace bm

