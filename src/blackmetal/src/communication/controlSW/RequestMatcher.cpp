#include "RequestMatcher.hpp"

// Robot has timer set to 10 seconds. After this time the robot stops.
#define ROBOT_TIMER_LIMIT 10s

RequestMatcher::RequestMatcher(const std::pair<double, double> &speeds)
	// Set the time to 10 seconds before now, so the first message is always send.
	: m_time(std::chrono::system_clock::now() - ROBOT_TIMER_LIMIT)
	, m_speeds(speeds)
	, m_sendStatus(bm::Status::OK)
{}

bool RequestMatcher::checkLastInstance(const std::pair<double, double> &speeds)
{
	auto now = std::chrono::system_clock::now();

	std::scoped_lock<std::mutex> lock(m_lock);
	if (m_speeds == speeds) {
		// We do not want to wate for the robot to stop.
		if (m_time + 9s < now) {
			m_time = now;
			return false;
		}
		if (m_sendStatus != bm::Status::OK) {
			m_time = now;
			return false;
		}
		return true;
	}

	m_time = now;
	return false;
}

RequestMatcher &RequestMatcher::setSendSpeeds(const std::pair<double, double> &speeds)
{
	std::scoped_lock<std::mutex> lock(m_lock);
	m_speeds = speeds;
	return *this;
}

RequestMatcher &RequestMatcher::setSendStatus(bm::Status status)
{
	std::scoped_lock<std::mutex> lock(m_lock);
	m_sendStatus = status;
	return *this;
}

