#pragma once

#include "ReturnStatus.hpp"
#include "queue/Queue.hpp"

#include <string>
#include <variant>
#include <mutex>

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
	/// Default constructor
	Client();
	/**
	 * @brief This constructor starts with calling the default constructor.
	 * The set function is called and the worker thread is created and detached.
	 *
	 * @see Client::Client() Default constructor.
	 * @see Client::start() Starts the client and connects to the server.
	 * @see Client::workerThread() Sends the data send and received from the server.
	 *
	 * @param port Port number to connect to.
	 * @param address IPv4 address to connect to.
	 */
	Client(int port, const std::string &address);
	/// Destructor
	virtual ~Client();

	/**
	 * @brief Starts the client and connects to the specified server IP and port.
	 * The send and receive blocking functions are limited to a specified time defined in macro
	 * WAIT_TIME (defined in the source file).
	 *
	 * @param port Port to connect to.
	 * @param address Address to connect to.
	 */
	void start(int port, const std::string &address);

	/**
	 * @brief Disconnects the client from the server. And sets the connection status to false.
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
	 * @brief Forms a json string out of supplied parameters and pushes them to the queue.
	 *
	 * NOTE: The function does not need to be virtual, although if there was a case where the execute function
	 * must be different for some case the class could be easily inherited. However, the user is strongly
	 * encouraged to use the supplied thread safe queue.
	 *
	 * @see bm::Command The enum class of possible request commands.
	 *
	 * @param cmd Command which should the robot execute.
	 * @param rightWheel Right wheel speed. This parameter is needed only in bm::Command::SET_LR_WHEEL_VELOCITY.
	 * @param leftWheel Right wheel speed. This parameter is needed only in bm::Command::SET_LR_WHEEL_VELOCITY.
	 */
	virtual void sendRequest(bm::Command cmd, double rightWheel = 0, double leftWheel = 0);

	/**
	 * @brief A specific function just for calling execute with bm::Command::SET_LR_WHEEL_VELOCITY.
	 *
	 * The return status is than evaluated in @see evalReturnState.
	 *
	 * @param rightWheel Right wheel speed.
	 * @param leftWheel Right wheel speed.
	 */
	void request(double rightWheel, double leftWheel);

	/**
	 * @brief Add the request to priority queue to be sent to server.
	 * @see ts::Queue Thread safe queue managing the requests send to the robot.
	 *
	 * @param msg Json string to be sent to the server.
	 */
	void enqueue(const std::string &msg);

	/**
	 * @brief Returns the first json message that is located in the queue.
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
	 * @brief Send a desired string message to the server.
	 *
	 * This function is used only internally in a separate thread. The way to send the message
	 * to the server is by enqueueing the message. The thread will then send it. Be aware
	 * that the used queue is priority queue. The priority have messages setting the robot's
	 * left and right wheel speed. 
	 *
	 * @param msg message to be send.
	 * @return bm::Status::SEND_ERROR when the ::send function crashes, bm::Status::OK otherwise.
	 */
	bm::Status send(const std::string &msg);

	/**
	 * @brief Receive a message from the server.
	 *
	 * Be aware this is a blocking function. The execution will stop until the server does not
	 * send a message to us. The only data that could be used from the server are the wheel velocities.
	 * You can get them using Client::robotVelocity method.
	 *
	 * @see robotVelocity public method for receiving the json containing robot's left and right wheel speed.
	 * @see ts::Queue Thread safe queue managing the requests send to the robot.
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

	/**
	 * @brief Function that evaluates the request status.
	 *
	 * The robot always returns a json that indicates whether the request was successful
	 * or the robot's buffer is full.
	 *
	 * @param returnJson Json string returned from the communication.
	 */
	bm::Status evalReturnState(const std::string &returnJson);

private:
	/// IP address to which we tried or are connected to.
	std::string m_address;
	/// File descriptor of the communication.
	int m_clientFD;
	/// Flag checking the client's connection.
	bool m_connected;
	/// Port to which is the client connected to.
	int m_port;
	/// Socket for biding to server, sending and receiving data.
	int m_socket;
	/// Thread safe priority queue managing the json string that are to be sent to the robot.
	ts::Queue m_queue;
	mutable std::mutex m_mutex;
	/// Thread safe queue containing the returned left and right wheel velocity.
	std::queue<std::string> m_odometryMessages;
};

