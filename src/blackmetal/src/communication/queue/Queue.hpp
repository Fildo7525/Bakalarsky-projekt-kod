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
 * @brief Priority queue.
 *
 * The priority queue is implemented using std::priority_queue.
 * The ordering is done using the callable object std::greater<T>().
 *
 * @tparam T The type of the elements stored in the queue.
 */
template <typename T>
using pqueue = std::priority_queue<T, std::vector<T>, std::greater<T>>;

/**
 * @brief ts is a namespace grouping thread safe classes and functions.
 */
namespace ts
{


/**
 * @class Queue
 * @brief Thread safe dynamic templated priority queue.
 *
 * The queue is implemented using @c pqueue<T>, @c std::mutex and @c std::condition_variable.
 * When you want to pop an element from the queue the thread will be blocked until you push
 * an element from other thread.
 *
 * @see pqueue<T> For more information about the priority queue.
 *
 * @tparam T The type of the elements stored in the queue.
 */
template <typename T>
class Queue
{
	/**
	 * @brief Checks if the queue is empty.
	 *
	 * Moved out of public interface to prevent races between this
	 * and pop().
	 *
	 * @return true if the queue is empty, false otherwise.
	 */
	bool empty();

	/**
	 * @brief The class is not default constructable, copyable or assignable.
	 * @{
	 */
	Queue() = delete;
	Queue(const Queue &) = delete;
	Queue& operator=(const Queue &) = delete;
	/**
	 * @}
	 */

public:
	/**
	 * @brief Default constructor.
	 *
	 * @param name The name of the queue used for debugging.
	 */
	explicit Queue(const std::string &name);

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
	 * is evaluated by the @c std::greater<T>() as grater
	 * the item is prioritized.
	 *
	 * @param item to be added to the queue.
	 */
	void push(const T &item);

private:
	/// Priority queue containing the requests to be send.
	pqueue<T> m_pqueue;

	/// The queue mutex.
	std::mutex m_qMutex;

	/// Condition variable ensuring the pop method will wait for an element if the queue is empty.
	std::condition_variable m_cvPush;

	/// The name of the queue used for debugging.
	const std::string m_queueName;
};

} // namespace ts

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

