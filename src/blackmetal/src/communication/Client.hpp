#pragma once

#include "ReturnStatus.hpp"

#include <string>
#include <variant>

/**
 * @class Client
 * @brief Base class handeling TCP/IP connections
 *
 * The classes inputs are prot and address of the desired server.
 * This class than offers multiple methos of communication.
 */
class Client
{
public:
	/// Default constructor
	Client() = default;
	/// Constructor
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
	 * @brief Get the string represnetation of the bm::Status code.
	 *
	 * @param status Status to be transformed.
	 */
	static std::string stringifyStatus(const bm::Status status);

	/**
	 * @brief Get the string represnetation of the bm::Command code.
	 *
	 * @param command Status to be transformed.
	 */
	static std::string stringifyCommand(const bm::Command command);

	/**
	 * @brief Forms a json string out of supplied parameters and sends them to server.
	 *
	 * @param cmd @see Command which should the robot execute.
	 * @param rightWheel Right wheel speed. This parameter is needed only in bm::Command::SET_LR_WHEEL_VELOCITY.
	 * @param leftWheel Right wheel speed. This parameter is needed only in bm::Command::SET_LR_WHEEL_VELOCITY.
	 * @return A variant containing either an error status 
	 * 		   bm::Status::SEND_ERROR when the ::send function crashes,
	 * 		   bm::Status::RECEIVE_ERROR when the ::send function crashes,
	 * 		   returned std::string message otherwise.
	 */
	virtual std::variant<bm::Status, std::string> execute(bm::Command cmd, int rightWheel = 0, int leftWheel = 0);

	/**
	 * @brief A specific function just for calling execute with bm::Command::SET_LR_WHEEL_VELOCITY.
	 *
	 * The return status is than evaluated in @see evalReturnState.
	 *
	 * @param rightWheel Right wheel speed.
	 * @param leftWheel Right wheel speed.
	 * @return bm::Status::SEND_ERROR when the ::send function crashes,
	 * 		   bm::Status::RECEIVE_ERROR when the ::send function crashes,
	 * 		   bm::Status::OK otherwise.
	 */
	bm::Status request(int rightWheel, int leftWheel);

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
	 * @brief Pure virtual function that evaluates the request status.
	 *
	 * @param returnJson Json string returned from the communication.
	 */
	virtual bm::Status evalReturnState(const std::string &returnJson) = 0;

private:
	/// Ip addres to which we tried or are connected to.
	std::string m_address;
	/// Socket for biding to server, sending and receiving data.
	int m_socket;
	/// Flach chacking the client's connection.
	bool m_connected;
};

