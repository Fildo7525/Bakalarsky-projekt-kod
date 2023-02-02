#pragma once

#include "ReturnStatus.hpp"
#include "Queue.hpp"

#include <string>
#include <variant>
#include <mutex>

/**
 * @class Client
 * @brief Base class handling TCP/IP connections
 *
 * The classes inputs are port and address of the desired server.
 * This class then offers multiple methods of communication with
 * the mobile robot. The communication is handled by json strings.
 *
 * The order of send and received messages is organized using a thread
 * safe queue.
 * @see ts::Queue thread safe queue.
 *
 * NOTE:
 * The strings cannot be passed to parser, because the messages we
 * receive are no according to the standard. We have to do it manually.
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
	 * @brief Forms a json string out of supplied parameters and sends them to server.
	 *
	 * The request consists of either double or long parameters. The double parameters
	 * are used in set velocity request. The long parameters are used in the set position request.
	 *
	 * @see bm::Command The enumeration class of possible request commands.
	 * @see WheelValueT The variant parameter of the request.
	 * @see requestSpeed The function for setting the robot speed.
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
	 * @brief Add the request to priority queue to be sent to server.
	 * @see ts::Queue Thread safe queue managing the requests send to the robot.
	 *
	 * @param msg Json string to be sent to the server.
	 */
	void enqueue(const std::string &msg);

	/**
	 * @brief Returns the first json message that is located in the odometry queue.
	 */
	std::string robotVelocity();

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
	 * This function is called in case that all the communication
	 * was successful and we received data from the server.
	 *
	 * @param returnJson Json string returned from the communication.
	 */
	virtual bm::Status evalReturnState(const std::string &returnJson);

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

private:
	/**
	 * @brief Method handling the json requests.
	 *
	 * The method runs in different thread detached from the main thread. Handles the requests
	 * pushed to the priority queue. Uses the send and receive functions. The returned messages
	 * containing the robot velocity is stored in m_odometryMessages queue. 
	 */
	void workerThread();

private:
	/// IP address to which we tried or are connected to.
	std::string m_address;

	/// Flag checking the client's connection.
	bool m_connected;

	/// Port to which is the client connected to.
	int m_port;

	/// Thread safe priority queue managing the json string that are to be sent to the robot.
	std::shared_ptr<ts::Queue> m_queue;

	/// Thread safe queue containing the returned left and right wheel velocity.
	std::shared_ptr<ts::Queue> m_odometryMessages;

	/// Socket for biding to server, sending and receiving data.
	int m_socket;
};
