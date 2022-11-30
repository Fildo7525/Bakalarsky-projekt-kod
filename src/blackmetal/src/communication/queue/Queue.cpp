#include "Queue.hpp"

#include "log.hpp"

#include <mutex>
#include <queue>
#include <iterator>
#include <algorithm>


INIT_MODULE(Queue, dbg_level::DBG);

std::ostream& ts::operator<<(std::ostream &os, const ts::my_queue &queue)
{
	std::copy(queue.c.cbegin(),
			  queue.c.cend(),
			  std::ostream_iterator<std::string>(os, "\n\t"));
	return os;
}


ts::Queue::Queue(const std::string &name)
	: m_queueName(name)
{
}

bool ts::Queue::empty() const
{
	std::scoped_lock<std::mutex> lock(m_qMutex);
	return m_pqueue.empty();
}

unsigned long ts::Queue::size() const
{
	std::scoped_lock<std::mutex> lock(m_qMutex);
	return m_pqueue.size();
}

std::string ts::Queue::pop()
{
	// DBG("Pop was invoked");
	std::string tmp;

	if (empty()) {
		std::unique_lock<std::mutex> lk(m_cvMutex);
		FATAL("The queue " << m_queueName << " will wait until the queue has any data to pop");
		m_cvPush.wait(lk, [this] { return !this->empty(); });
		SUCCESS("The wating was canceled in " << m_queueName);
	}

	WARN("Locking qMutex in " << m_queueName);
	std::unique_lock<std::mutex> lk(m_qMutex);
	INFO("popping from " << m_queueName);
	INFO("Size of " << m_queueName << " is " << m_pqueue.size());
	SUCCESS("The contents of " << m_queueName << " are: " << m_pqueue);
	tmp = m_pqueue.top();
	INFO("The popped item is " << tmp);
	m_pqueue.pop();
	return tmp;
}

void ts::Queue::push(const std::string &item)
{
	WARN("Locking the qMutex in " << m_queueName);
	std::scoped_lock<std::mutex> lock(m_qMutex);
	INFO("pushing data to " << m_queueName);
	m_pqueue.push(item);
	INFO(item << " was pushed to queue " << m_queueName);
	{
		std::scoped_lock<std::mutex> lk(m_cvMutex);
		SUCCESS("The contents of " << m_queueName << " are: " << m_pqueue);
		m_cvPush.notify_one();
	}
	WARN("Notifying the pop function");
}

