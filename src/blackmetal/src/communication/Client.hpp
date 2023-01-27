#pragma once

#include "ReturnStatus.hpp"

#include <string>
#include <variant>
#include <mutex>

// wait for 200ms
#define WAIT_TIME 200'000

/**
 * @class Client
 * @brief Base class handling TCP/IP connections
 *
 * The classes inputs are port and address of the desired server.
 * This class than offers multiple methods of communication.
 */
class Client
{
public:
	/**
	 * The sendRequest function takes arguments for left and right wheel
	 * for setting position and velocity. The velocity is set with double
	 * and the position is set with long. This is the generic solution for this issue.
	 */
	using WheelValueT = std::variant<long, double>;

	/**
	 * @brief Default constructor
	 */
	Client() = default;

	/**
	 * @brief Constructs and initializes the client for communication
	 *
	 * @param port On which to start the communication. 
	 * @param address Address of the server.
	 */
	Client(int port, const std::string &address);

	/// Destructor
	virtual ~Client();

	/**
	 * @brief Starts the client and connects to the specified server IP.
	 *
	 * @param port Port to connect to.
	 * @param address Address to connect to.
	 */
	void start(int port, const std::string &address);

	/**
	 * @brief Disconnects the client from the server.
	 */
	void stop();

	/**
	 * @brief Get the string representation of the bm::Status code.
	 *
	 * @param status Status to be transformed.
	 */
	static std::string stringifyStatus(const bm::Status status);

	/**
	 * @brief Get the string representation of the bm::Command code.
	 *
	 * @param command Status to be transformed.
	 */
	static std::string stringifyCommand(const bm::Command command);

	/**
	 * @brief Forms a json string out of supplied parameters and sends them to server.
	 *
	 * The request consists of either double or long parameters. The double parameters
	 * are used in set velocity request. The long parameters are used in the set position request.
	 *
	 * @see bm::Command The enumeration class of possible request commands.
	 * @see WheelValueT The variant parameter of the request.
	 * @see request The function for setting the robot speed.
	 *
	 * @param cmd Command which should the robot execute.
	 * @param rightWheel Right wheel speed or position. This parameter is needed only in
	 * bm::Command::SET_LR_WHEEL_VELOCITY and bm::Command::SET_LR_WHEEL_POSITION.
	 * @param leftWheel Right wheel speed or position. This parameter is needed only in
	 * bm::Command::SET_LR_WHEEL_VELOCITY and bm::Command::SET_LR_WHEEL_POSITION.
	 */
	virtual bm::Status sendRequest(bm::Command cmd, WheelValueT rightWheel = 0, WheelValueT leftWheel = 0);

	/**
	 * @brief A specific function just for calling execute with bm::Command::SET_LR_WHEEL_VELOCITY.
	 *
	 * The return status is than evaluated in @see evalReturnState.
	 *
	 * @param rightWheel Right wheel speed.
	 * @param leftWheel Right wheel speed.
	 * @return bm::Status::SEND_ERROR when the ::send function crashes,
	 * 		   bm::Status::RECEIVE_ERROR when the ::read function crashes,
	 * 		   bm::Status::OK otherwise.
	 */
	bm::Status requestSpeed(double rightWheel, double leftWheel);

	/**
	 * @brief A specific function just for calling execute with bm::Command::SET_LR_WHEEL_POSITION.
	 *
	 * The return status is than evaluated in evalReturnState.
	 *
	 * @see evalReturnState Evaluation method for the confirmation of request.
	 *
	 * @param rightWheel Right wheel whished position.
	 * @param leftWheel Right wheel whished position.
	 */
	bm::Status requestPosition(long rightWheel, long leftWheel);

	/**
	 * @brief Send a desired string message to the server.
	 *
	 * @param msg message to be send.
	 * @return bm::Status::SEND_ERROR when the ::send function crashes, bm::Status::OK otherwise.
	 */
	bm::Status send(const std::string &msg);

	/**
	 * @brief Receive a message from the server.
	 *
	 * Be aware this is a blocking function.
	 * The execution will stop until the server does not send a message to us.
	 *
	 * @param msg Variable where the received message should be saved.
	 * @return bm::Status::RECEIVE_ERROR when the ::read function crashes, bm::Status::OK otherwise.
	 */
	bm::Status receive(std::string &msg);

	/**
	 * @brief Returns the address with which was the start method called with.
	 */
	std::string address();

	/**
	 * @brief Returns a copy of socket file descriptor.
	 */
	int socketFD();

	/**
	 * @brief Indicates if we are or are not connected.
	 */
	bool connected();

protected:
	/**
	 * @brief Virtual function that evaluates the request status.
	 *
	 * @param returnJson Json string returned from the communication.
	 */
	virtual bm::Status evalReturnState(const std::string &returnJson);

private:
	/// IP address to which we tried or are connected to.
	std::string m_address;

	/// File descriptor of the communication.
	int m_clientFD;

	/// Flag checking the client's connection.
	bool m_connected;

	/// Port to which is the client connected to.
	int m_port;

	/// Synchronizes threads on receiving a request to server.
	std::mutex m_receiveSynchronizer;

	/// Synchronizes threads on sending a request to server.
	std::mutex m_sendSynchronizer;

	/// Socket for biding to server, sending and receiving data.
	int m_socket;
};

