#pragma once

#include <string>
#include <variant>

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
	EMG_SOPT,
	/// Normal stop
	NORMAL_STOP,
	/// Sets Left and right wheel velocity.
	/// The velocities are required for this command.
	SET_LR_WHEEL_VELOCITY,
	/// Not implemented.
	NONE_4,
	/// Not implemented.
	NONE_5,
	/// Get left and right wheel velocities.
	GET_LR_WHEEL_VELOCITY,
	/// Prepare the controller of left and right wheel.
	PREPARE_WHEEL_CONTROLLER,
};

/**
 * @enum Status
 * @brief return status of the server.
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
	/// The response could not be recieved.
	RECIEVE_ERROR,
};

} // namespace bm


class Client
{
public:
	Client(int port, const std::string &address);
	virtual ~Client();

	virtual std::variant<bm::Status, std::string> execute(bm::Command cmd, int rightWheel, int leftWheel);
	bm::Status request(int rightWheel, int leftWheel);

	std::string receive();

	int socketFD();
private:
	bm::Status evalReturnState(const std::string &returnJson);

private:
	/// File descriptor of the object.
	int m_clientFD;
	/// Socket for biding to server, sending and recieving data.
	int m_socket;
};
