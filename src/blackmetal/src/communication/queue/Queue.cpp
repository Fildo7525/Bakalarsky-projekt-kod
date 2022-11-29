#include "Queue.hpp"

#include "log.hpp"

#include <mutex>
#include <queue>
#include <iterator>
#include <algorithm>


INIT_MODULE(Queue, dbg_level::DBG);

ts::Queue::Queue(const std::string &name)
	: m_queueName(name)
{
}

bool ts::Queue::empty() const
{
	// INFO("The queue is empty " << m_pqueue.empty());
	WARN("LOCKING m_qMutex");
	std::lock_guard<std::mutex> lock(m_qMutex);
	return m_pqueue.empty();
}

unsigned long ts::Queue::size() const
{
	WARN("LOCKING m_qMutex");
	std::lock_guard<std::mutex> lock(m_qMutex);
	// INFO("The queue has size " << m_pqueue.size());
	return m_pqueue.size();
}

std::string ts::Queue::pop()
{
	// DBG("Pop was invoked");
	std::string tmp("");
	bool empty;
	{
		WARN("LOCKING m_qMutex");
		std::lock_guard<std::mutex> lock(m_qMutex);
		empty = m_pqueue.empty();
	}

	if (empty) {
		m_readyToPush = false;
		WARN("LOCKING m_communicationMutex");
		std::unique_lock<std::mutex> lk(m_communicationMutex);
		WARN("The queue " << m_queueName << " will wait until the queue has any data to pop");
		m_cvPush.wait(lk, [this] { return this->m_readyToPush; });
	}

	{
		WARN("LOCKING m_qMutex");
		std::lock_guard<std::mutex> lock(m_qMutex);
		tmp = m_pqueue.top();
	}
	// INFO("Returning " << tmp);
	{
		WARN("LOCKING m_qMutex");
		std::lock_guard<std::mutex> lock(m_qMutex);
		m_pqueue.pop();
	}
	return tmp;
}

void ts::Queue::push(const std::string &item)
{
	// DBG("Push was invoked");
	bool empty = false;
	if (m_pqueue.size() == 0) {
		WARN("The queue " << m_queueName << " is empty, thus the pop will be notified");
		empty = true;
	}

	{
		std::lock_guard<std::mutex> lock(m_qMutex);
		m_pqueue.push(item);
	}
	INFO(item << " was pushed to queue " << m_queueName);

	if (empty) {
		WARN("LOCKING m_communicationMutex");
		std::lock_guard<std::mutex> lk(m_communicationMutex);
		m_readyToPush = true;
		WARN("Notifying the pop function");
		m_cvPush.notify_one();
	} else {
		WARN("The queue was not empty");
	}
}

