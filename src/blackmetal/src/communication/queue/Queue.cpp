#include "Queue.hpp"
#include <mutex>
#include <queue>
#include <iterator>
#include <algorithm>

bool readyToPush = true;

bool ts::Queue::empty() const
{
	return m_pqueue.empty();
}

unsigned long ts::Queue::size() const
{
	std::lock_guard<std::mutex> lock(m_qMutex);
	return m_pqueue.size();
}

std::string ts::Queue::pop()
{
	std::lock_guard<std::mutex> lock(m_qMutex);
	std::string tmp("");

	if (m_pqueue.empty()) {
		readyToPush = false;
		std::unique_lock<std::mutex> lk(m_communicationMutex);
		m_cvPush.wait(lk, [] {return readyToPush; });
	}

	tmp = m_pqueue.top();
	m_pqueue.pop();
	return tmp;
}

void ts::Queue::push(const std::string &item)
{
	bool empty = false;
	std::lock_guard<std::mutex> lock(m_qMutex);
	if (m_pqueue.size() == 0) {
		empty = true;
	}

	m_pqueue.push(item);

	if (empty) {
		std::lock_guard<std::mutex> lk(m_communicationMutex);
		readyToPush = true;
		m_cvPush.notify_one();
	}
}

