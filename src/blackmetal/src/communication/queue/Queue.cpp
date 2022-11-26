#include "Queue.hpp"
#include <mutex>

bool readyToPush = true;

bool ts::Queue::empty() const
{
	return m_queue.empty();
}

ts::Queue::Queue(unsigned long maxSize)
	: m_maxSize(maxSize)
	, m_queue()
{
}

unsigned long ts::Queue::size() const
{
	std::lock_guard<std::mutex> lock(m_qMutex);
	return m_queue.size();
}

std::string ts::Queue::pop()
{
	std::lock_guard<std::mutex> lock(m_qMutex);
	std::string tmp("");
	bool full = m_maxSize == m_queue.size();

	if (m_queue.empty()) {
		readyToPush = false;
		std::unique_lock<std::mutex> lk(m_communicationMutex);
		m_cvPush.wait(lk, [] {return readyToPush; });
	}

	tmp = m_queue.front();
	m_queue.pop();

	if (full) {
		std::lock_guard<std::mutex> lk(m_communicationMutex);
		readyToPush = true;
		m_cvPush.notify_one();
	}
	return tmp;
}

void ts::Queue::push(const std::string &item)
{
	bool empty = false;
	std::lock_guard<std::mutex> lock(m_qMutex);
	if (m_maxSize == m_queue.size() ) {
		readyToPush = false;
		std::unique_lock<std::mutex> lk(m_communicationMutex);
		m_cvPush.wait(lk, [] {return readyToPush; });
	}
	if (m_maxSize == 0) {
		empty = true;
	}

	m_queue.push(item);

	if (empty) {
		std::lock_guard<std::mutex> lk(m_communicationMutex);
		readyToPush = true;
		m_cvPush.notify_one();
	}
}

