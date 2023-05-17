#pragma once

#include "Client.hpp"
#include "controlSW/RequestMatcher.hpp"
#include "types/RobotRequestType.hpp"
#include "types/RobotResponseType.hpp"

/// Constatns used to convert the impulses to meters per second.
/// They were obtained by tests and than by applying the linear regression to the measured samples.
#define FROM_IMP_TO_MPS_L 1198.86351909
#define FROM_IMP_TO_MPS_R 1212.51934985

#include <functional>

/**
 * @class RobotDataDelegator
 * @brief Child class implementing Client class and operating with robot messages.
 *
 * This class sends json requests via the @c Client::send() method. The parameters to send are supplied
 * in method sendRequest. Method @c receive receives data from robot. Evaluetes the data and pushes them
 * to separate ts::Queue so that Odometry class could take them. If the received data consist of multiple
 * messages the message is split and every resonse is evaluated separatly. All this takes place
 * in separate thread.
 *
 * The order of send and received messages is organized using a thread
 * safe queue.
 * @see ts::Queue thread safe queue.
 * @see Client class used for communication with the robot.
 * @see RobotRequestType Class used for storing the request parameters.
 * @see RobotResponseType Class used for storing the response parameters.
 *
 * @note: The strings cannot be passed to parser, because the messages we
 * receive are no according to the standard. We have to do it manually.
 */
class RobotDataDelegator
	: public Client
{
public:

	/**
	 * @brief Constructor implementing @c Client constructor.
	 *
	 * @param port Port to connect to.
	 * @param address IPv4 Address of the server.
	 */
	RobotDataDelegator(int port, const std::string &address, std::shared_ptr<RequestMatcher> matcher);

	/**
	 * @brief Forms a json string out of supplied parameters and enqueues them.
	 *
	 * The request consists of either double or long parameters. The double parameters
	 * are used in set velocity request. The long parameters are used in the set position request.
	 *
	 * @see bm::Command The enumeration class of possible request commands.
	 * @see RobotRequestType::WheelValueT The variant parameter of the request.
	 * @see requestSpeed The function for setting the robot speed.
	 *
	 * @param cmd Command which should the robot execute.
	 * @param rightWheel Right wheel speed or position. This parameter is needed only in
	 * @c bm::Command::SET_LR_WHEEL_VELOCITY and @c bm::Command::SET_LR_WHEEL_POSITION.
	 * @param leftWheel Right wheel speed or position. This parameter is needed only in
	 * @c bm::Command::SET_LR_WHEEL_VELOCITY and @c bm::Command::SET_LR_WHEEL_POSITION.
	 *
	 * @return Always returns @c bm::Status::OK after the request is enqueud.
	 */
	virtual bm::Status sendRequest(bm::Command cmd, RobotRequestType::WheelValueT rightWheel = 0, RobotRequestType::WheelValueT leftWheel = 0);

	/**
	 * @brief This function calls the @c sendRequest with @c bm::Command::SET_LR_WHEEL_VELOCITY.
	 *
	 * @param rightWheel Right wheel speed.
	 * @param leftWheel Right wheel speed.
	 * @return Always returns @c bm::Status::OK after the request is enqueud.
	 */
	bm::Status requestSpeed(double rightWheel, double leftWheel);

	/**
	 * @brief This function calls the @c sendRequest with @c bm::Command::SET_LR_WHEEL_POSITION.
	 *
	 * @param rightWheel Right wheel wished position.
	 * @param leftWheel Right wheel wished position.
	 * @return Always returns @c bm::Status::OK after the request is enqueud.
	 */
	bm::Status requestPosition(long rightWheel, long leftWheel);

	/**
	 * @brief Returns the first json message that is located in the odometry queue.
	 *
	 * @note the queue is thread safe and will block until the message is available.
	 * @see ts::Queue Thread safe queue managing the recevied messages.
	 *
	 * @return The first json message in the priority queue.
	 */
	RobotResponseType robotVelocity();

	/**
	 * @brief Sets the callabck function for resetting the filter when the robot velocity changes.
	 *
	 * @param onVelocityChange Callabck function called when the robot velocity changes.
	 */
	void setOnVelocityChangeCallback(std::function<void(RobotRequestType)> onVelocityChange);

private:
	/**
	 * @brief Check if the @c receive did not read two messags at the same time.
	 *
	 * There may happen that the robot will send us two messages faster than
	 * we can read. In that case parse the received string and split the responses.
	 * Validate both of them.
	 *
	 * @param msg Received message from the robot.
	 *
	 * @return Vector of strings containing the responses.
	 */
	std::vector<std::string> splitResponses(const std::string &msg);

	/**
	 * @brief Add the request to priority queue to be sent to server.
	 *
	 * @see ts::Queue Thread safe queue managing the requests send to the robot.
	 *
	 * @param msg Json string to be sent to the server.
	 */
	void enqueue(const RobotRequestType &msg);

	/**
	 * @brief Overridden function from @c Client.
	 *
	 * This function detects whether the received messages are read at once.
	 * If multiple messages were received on one read the function will evaluate them both.
	 *
	 * @see Client::evalReturnState Function evaluating the received message state.
	 *
	 * @param msg String to where should the received string be saved.
	 * @return If the @c Client::receive() crashes the error is forwarded,
	 * 		   @c bm::Status::MULTIPLE_RECEIVE if the multiple messages were received,
	 * 		   @c bm::Status::OK otherwise.
	 */
	bm::Status receive(std::string &msg) override;

	/**
	 * @brief Method handling the json requests.
	 *
	 * The method runs in different thread detached from the main thread. Handles the requests
	 * pushed to the priority queue. Uses the @c send and @c receive methods. The returned messages
	 * containing the robot velocity is stored in @c m_odometryMessages queue. 
	 */
	void workerThread();

private:
	/// Thread safe priority queue managing the json string that are to be sent to the robot.
	std::shared_ptr<ts::Queue<RobotRequestType>> m_queue;

	/// Thread safe queue containing the returned left and right wheel velocity.
	std::shared_ptr<ts::Queue<RobotResponseType>> m_odometryMessages;

	/// Object for filtering the duplicated requests.
	std::shared_ptr<RequestMatcher> m_matcher;

	/// Flag for detecting the change of the robot velocity. When set the filter will be reset using the @c m_onVelocityChange callback.
	bool m_velocityChangeFlag;

	/// Callback for resetting the filter when the robot velocity changes.
	std::function<void(RobotRequestType)> m_onVelocityChange;
};

