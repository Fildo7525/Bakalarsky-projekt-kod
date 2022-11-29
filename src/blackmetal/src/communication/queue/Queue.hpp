#pragma once

#include <condition_variable>
#include <queue>
#include <string>
#include <mutex>
#include <vector>

/**
 * @brief ts is a namespace grouping thread safe classes and functions.
 */
namespace ts
{

/**
 * @class Queue
 * @brief Thread safe dynamic priority queue storing json strings
 * that are to be send to the desired IP address. The preferred strings
 * are those that are setting the robot left and right wheel speed.
 * The ordering is done using the callable object std::greater<std::string>()
 */
class Queue {
	/**
	 * @brief Checks if the queue is empty.
	 *
	 * Moved out of public interface to prevent races between this
	 * and pop().
	 *
	 * @return true if the queue is empty, false otherwise.
	 */
	bool empty() const;

public:
	/**
	 * @brief Default constructor.
	 */
	explicit Queue(const std::string &name);

	/**
	 * @brief The class is not copyable.
	 */
	Queue() = delete;
	Queue(const Queue &) = delete;
	Queue& operator=(const Queue &) = delete;
 
	/**
	 * @brief Returns the size of the priority queue.
	 */
	unsigned long size() const;

	/**
	 * @brief Returns the top element in the priority queue.
	 */
	std::string pop();

	/**
	 * @brief Add a new element to the priority queue.
	 *
	 * Inserts the element to the queue. If the inserted item @param item 
	 * is sending request for setting the left and right wheel speed
	 * prioritize this request.
	 *
	 * @param item to be added to the queue.
	 */
	void push(const std::string &item);

private:
	/// Priority queue containing the requests to be send.
	std::priority_queue<std::string, std::vector<std::string>, std::greater<std::string>> m_pqueue;
	/// The queue mutex.
	mutable std::mutex m_qMutex;
	/// Mutex activated in the communication methods.
	mutable std::mutex m_communicationMutex;
	/// Condition variable ensuring the pop method will wait for an element if the queue is empty.
	std::condition_variable m_cvPush;
};

} // thread safe

