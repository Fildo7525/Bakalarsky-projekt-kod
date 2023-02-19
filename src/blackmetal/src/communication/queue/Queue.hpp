#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iterator>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

/**
 * @brief ts is a namespace grouping thread safe classes and functions.
 */
namespace ts
{

template <typename T>
using pqueue = std::priority_queue<T, std::vector<T>, std::greater<T>>;

/**
 * @class Queue
 * @brief Thread safe dynamic priority queue storing json strings
 * that are to be sent to the desired IP address. The preferred strings
 * are those that are setting the robot's left and right wheel speed.
 * The ordering is done using the callable object std::greater<std::string>()
 */
template <typename T>
class Queue {
	/**
	 * @brief Checks if the queue is empty.
	 *
	 * Moved out of public interface to prevent races between this
	 * and pop().
	 *
	 * @return true if the queue is empty, false otherwise.
	 */
	bool empty();

public:
	/**
	 * @brief Default constructor.
	 */
	explicit Queue(const std::string &name);

	/**
	 * @brief The class is not default constructable, copyable or assignable.
	 */
	Queue() = delete;
	Queue(const Queue &) = delete;
	Queue& operator=(const Queue &) = delete;
 
	/**
	 * @brief Returns the size of the priority queue.
	 */
	unsigned long size();

	/**
	 * @brief Returns the top element in the priority queue and removes
	 * it from the internal structure.
	 */
	T pop();

	/**
	 * @brief Returns the copy of the top element in the priority queue.
	 */
	T peek();

	/**
	 * @brief Add a new element to the priority queue.
	 *
	 * Inserts the element to the queue. If the inserted item
	 * is evaluated by the std::greater<T>() as grater
	 * the item is prioritized.
	 *
	 * @param item to be added to the queue.
	 */
	void push(const T &item);

private:
	/// Priority queue containing the requests to be send.
	ts::pqueue<T> m_pqueue;

	/// The queue mutex.
	std::mutex m_qMutex;

	/// Condition variable ensuring the pop method will wait for an element if the queue is empty.
	std::condition_variable m_cvPush;

	// The name of the queue used for debugging.
	const std::string m_queueName;
};

} // thread safe namespace

template <typename T>
ts::Queue<T>::Queue(const std::string &name)
	: m_queueName(name)
{
}

template <typename T>
bool ts::Queue<T>::empty()
{
	std::scoped_lock<std::mutex> lock(m_qMutex);
	return m_pqueue.empty();
}

template <typename T>
unsigned long ts::Queue<T>::size()
{
	std::scoped_lock<std::mutex> lock(m_qMutex);
	return m_pqueue.size();
}

template <typename T>
T ts::Queue<T>::pop()
{
	// DBG("Pop was invoked");
	T tmp;

	std::unique_lock<std::mutex> lk(m_qMutex);
	m_cvPush.wait(lk, [this] { return !m_pqueue.empty(); });

	tmp = m_pqueue.top();
	m_pqueue.pop();
	return tmp;
}

template <typename T>
T ts::Queue<T>::peek()
{
	std::string tmp;

	std::unique_lock<std::mutex> lk(m_qMutex);
	m_cvPush.wait(lk, [this] { return !m_pqueue.empty(); });

	tmp = m_pqueue.top();
	return tmp;
}

template <typename T>
void ts::Queue<T>::push(const T &item)
{
	std::unique_lock<std::mutex> lock(m_qMutex);
	m_pqueue.push(item);
	m_cvPush.notify_one();
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const ts::Queue<T> &queue)
{
	std::thread([&os, &queue] {
		ts::pqueue<T> tmp;
		// Copy the contents of the queue to a temporary queue.
		// So that we will lock the mutex for as low time as possible.
		{
			std::scoped_lock<std::mutex> lock(queue.m_qMutex);
			tmp = queue.m_pqueue;
		}

		while (tmp.size() > 1)
		{
			os << tmp.top() << ", ";
			tmp.pop();
		}
	}).detach();
	return os;
}

