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
	std::lock_guard<std::mutex> lock(m_qMutex);
	return m_pqueue.empty();
}

unsigned long ts::Queue::size() const
{
	std::lock_guard<std::mutex> lock(m_qMutex);
	return m_pqueue.size();
}

std::string ts::Queue::pop()
{
	// DBG("Pop was invoked");
	std::string tmp;

	std::unique_lock<std::mutex> lk(m_qMutex);
	FATAL("The queue " << m_queueName << " will wait until the queue has any data to pop");
	m_cvPush.wait(lk, [this] { return this->m_pqueue.empty(); });
	SUCCESS("The wating was canceled in " << m_queueName);

	tmp = m_pqueue.top();
	m_pqueue.pop();
	return tmp;
}

void ts::Queue::push(const std::string &item)
{
	std::lock_guard<std::mutex> lock(m_qMutex);
	m_pqueue.push(item);
	INFO(item << " was pushed to queue " << m_queueName);
	m_cvPush.notify_one();
	WARN("Notifying the pop function");
}

