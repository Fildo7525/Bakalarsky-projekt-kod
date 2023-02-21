#pragma once

#include "ReturnStatus.hpp"
#include "Queue.hpp"

#include <string>
#include <variant>
#include <mutex>

constexpr long WAIT_TIME = 200'000;

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
	 * @brief Default constructor
	 */
	Client() = default;

	/**
	 * @brief Constructs and initializes the client for communication
	 *
	 * If -1 is supplied as a wateTime_usec the functions will stay blocking.
	 *
	 * @param port On which to start the communication. 
	 * @param address Address of the server.
	 * @param wateTime_usec How many seconds should the client wait on receive and send functions.
	 */
	Client(int port, const std::string &address, long wateTime_usec = WAIT_TIME);

	/// Destructor
	virtual ~Client();

	/**
	 * @brief Starts the client and connects to the specified server IP.
	 *
	 * @param port Port to connect to.
	 * @param address Address to connect to.
	 * @param wateTime_usec How many seconds should the client wait on receive and send functions.
	 * If -1 is supplied the functions will stay blocking.
	 */
	void start(int port, const std::string &address, long wateTime_usec);

	/**
	 * @brief Disconnects the client from the server.
	 */
	void stop();

	/**
	 * @brief Send a desired string message to the server.
	 *
	 * @param msg message to be send.
	 * @return bm::Status::SEND_ERROR when the ::send function crashes, bm::Status::OK otherwise.
	 */
	virtual bm::Status send(const std::string &msg);

	/**
	 * @brief Receive a message from the server.
	 *
	 * Be aware this is a blocking function.
	 * The execution will stop until the server does not send a message to us.
	 *
	 * @param msg Variable where the received message should be saved.
	 * @return bm::Status::RECEIVE_ERROR when the ::read function crashes, bm::Status::OK otherwise.
	 */
	virtual bm::Status receive(std::string &msg);

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

protected:
	/// IP address to which we tried or are connected to.
	std::string m_address;

	/// Flag checking the client's connection.
	mutable bool m_connected;

	/// Port to which is the client connected to.
	int m_port;

	/// Socket for biding to server, sending and receiving data.
	int m_socket;

	int m_clientFD;

	std::mutex m_mutex;
};

