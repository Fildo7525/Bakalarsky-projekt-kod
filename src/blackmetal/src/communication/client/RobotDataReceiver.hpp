#pragma once

#include "Client.hpp"
#include "types/RobotRequestType.hpp"

#include <functional>

/**
 * @class RobotDataReceiver
 * @brief Child class implementing Client class and operating with robot messages.
 *
 * This class send json requests via the Client's send method. The parameters to send are supplied
 * in method sendRequest. Method receive receives data from robot. Evaluetes the data and pushes them
 * to separate ts::Queue so that Odometry class could take them. If the received data consist of multiple
 * messages the message is split and every resonse is evaluated separatly. All this takes place
 * in separate thread.
 *
 * @see Client class used for communication with the robot.
 */
class RobotDataReceiver
	: public Client
{
public:

	/**
	 * @brief Constructor implementing Client constructor.
	 *
	 * @param port Port to connect to.
	 * @param address Address of the server.
	 */
	RobotDataReceiver(int port, const std::string &address);

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
	virtual bm::Status sendRequest(bm::Command cmd, RobotRequestType::WheelValueT rightWheel = 0, RobotRequestType::WheelValueT leftWheel = 0);

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
	 * @param rightWheel Right wheel wished position.
	 * @param leftWheel Right wheel wished position.
	 */
	bm::Status requestPosition(long rightWheel, long leftWheel);

	/**
	 * @brief Returns the first json message that is located in the odometry queue.
	 */
	std::string robotVelocity();

	void setOnVelocityChangeCallback(std::function<void()> onVelocityChange);

private:
	/**
	 * @brief Check if the read did not read two messags at the same time.
	 *
	 * There may happen that the robot will send us two messages faster than
	 * we can read. In that case parse the received string and split the responses.
	 * Validate both of them.
	 *
	 * @param msg received message from the robot.
	 */
	std::vector<std::string> splitResponses(const std::string &msg);

	/**
	 * @brief Add the request to priority queue to be sent to server.
	 * @see ts::Queue Thread safe queue managing the requests send to the robot.
	 *
	 * @param msg Json string to be sent to the server.
	 */
	void enqueue(const RobotRequestType &msg);

	/**
	 * @brief Overridden function from Client.
	 *
	 * This function detects whether the received messages are read at once.
	 * If multiple messages were received on one read the function will evaluate them both.
	 *
	 * @param msg String to where should the received string be saved.
	 */
	bm::Status receive(std::string &msg) override;

	/**
	 * @brief Method handling the json requests.
	 *
	 * The method runs in different thread detached from the main thread. Handles the requests
	 * pushed to the priority queue. Uses the send and receive functions. The returned messages
	 * containing the robot velocity is stored in m_odometryMessages queue. 
	 */
	void workerThread();

private:
	/// Thread safe priority queue managing the json string that are to be sent to the robot.
	std::shared_ptr<ts::Queue<RobotRequestType>> m_queue;

	/// Thread safe queue containing the returned left and right wheel velocity.
	std::shared_ptr<ts::Queue<std::string>> m_odometryMessages;

	/// Flag for detecting the change of the robot velocity. When set the filter will be reset using the m_onVelocityChange callback.
	bool m_resetFilter;

	/// Callback for resetting the filter when the robot velocity changes.
	std::function<void()> m_onVelocityChange;
};

