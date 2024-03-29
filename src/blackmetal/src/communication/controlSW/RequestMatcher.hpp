#pragma once

#include "ReturnStatus.hpp"

#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

/**
 * @class RequestMatcher
 * @brief Class used for filtering out the sending request data.
 *
 * If the requests parameters to be send are the same as the last send data
 * and the last send was successful. The message is not send. The robot
 * has set timer to stop execution after 10 seconds, therefore we set
 * a timelimit to 9 seconds. If this limit is overstepped we send
 * the next message even if it is a duplicate.
 */
class RequestMatcher
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param speeds Pair of doubles to be set as a default request parameters to be matched.
	 */
	explicit RequestMatcher(const std::pair<double, double> &speeds);

	/**
	 * @brief Checks if the message matches the last send message.
	 *
	 * @param msg Message to be matched.
	 * @return True if the @c msg is the last send message, false otherwise.
	 */
	bool checkLastInstance(const std::pair<double, double> &speeds);

	/**
	 * @brief Sets the last send message to be matched.
	 *
	 * @param msg String to be set as a last send message.
	 * @return Reference to this object.
	 */
	RequestMatcher &setSendSpeeds(const std::pair<double, double> &speeds);

	/**
	 * @brief Sets the last send status to be matched.
	 *
	 * @param status Status to be set as a last send status.
	 * @return Reference to this object.
	 */
	RequestMatcher &setSendStatus(bm::Status status);

private:
	/// Time of the last send message.
	std::chrono::system_clock::time_point m_time;

	/// Last send message.
	std::pair<double, double> m_speeds;

	/// Last send status.
	bm::Status m_sendStatus;

	std::mutex m_lock;
};

