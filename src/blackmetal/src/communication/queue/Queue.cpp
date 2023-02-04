#include "Queue.hpp"

#include "log.hpp"

#include <chrono>
#include <iomanip>
#include <mutex>
#include <queue>
#include <iterator>
#include <algorithm>

using namespace std::chrono_literals;

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

bool ts::Queue::empty()
{
	std::scoped_lock<std::mutex> lock(m_qMutex);
	return m_pqueue.empty();
}

unsigned long ts::Queue::size()
{
	std::scoped_lock<std::mutex> lock(m_qMutex);
	return m_pqueue.size();
}

std::string ts::Queue::pop()
{
	// DBG("Pop was invoked");
	std::string tmp;

	WARN(m_queueName << ": Locking m_qMutex");
	std::unique_lock<std::mutex> lk(m_qMutex);
	m_cvPush.wait(lk, [this] { return !m_pqueue.empty(); });

	SUCCESS("The contents of " << m_queueName << " are: " << m_pqueue);
	tmp = m_pqueue.top();
	INFO("The popped item is " << tmp);
	m_pqueue.pop();
	return tmp;
}

std::string ts::Queue::peek()
{
	std::string tmp;

	WARN(m_queueName << ": Locking m_qMutex");
	std::unique_lock<std::mutex> lk(m_qMutex);
	m_cvPush.wait(lk, [this] { return !m_pqueue.empty(); });

	SUCCESS("The contents of " << m_queueName << " are:\n\t" << m_pqueue);
	tmp = m_pqueue.top();
	return tmp;
}

void ts::Queue::push(const std::string &item)
{
	WARN("Locking the qMutex in " << m_queueName);
	std::unique_lock<std::mutex> lock(m_qMutex);
	INFO("pushing " << std::quoted(item) << " to " << m_queueName);
	m_pqueue.push(item);
	INFO("The contents of " << m_queueName << " are:\n\t" << m_pqueue);
	m_cvPush.notify_one();
	SUCCESS("Notifying the pop function");
}

