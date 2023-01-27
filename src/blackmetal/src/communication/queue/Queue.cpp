#include "Queue.hpp"

#include "log.hpp"

#include <chrono>
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

	WARN("Locking m_qMutex");
	std::unique_lock<std::mutex> lk(m_qMutex);
	while (m_pqueue.empty()) {
		FATAL("The queue " << m_queueName << " will wait until the queue has any data to pop");
		auto ret = m_cvPush.wait_for(lk, 300ms);
		if (ret == std::cv_status::timeout) {
			if (m_pqueue.empty()) {
				FATAL("Releasing lock. The queue is empty");
			}
			else {
				FATAL("The queue is NOT empty. Releasing and sending an empty string");
			}
			return "";
		}
		SUCCESS("The wating was canceled in " << m_queueName);
	}

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
	std::unique_lock<std::mutex> lock(m_qMutex);
	INFO("pushing " << item << " to " << m_queueName);
	m_pqueue.push(item);
	INFO("The contents of " << m_queueName << " are:\n\t" << m_pqueue);
	lock.unlock();
	m_cvPush.notify_one();
	SUCCESS("Notifying the pop function");
}

