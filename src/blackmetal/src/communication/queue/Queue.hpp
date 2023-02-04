#pragma once

#include <iostream>
#include <iterator>
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
 * @class my_queue
 * @brief Class accessing private members of std::priority_queue. NOT thread safe.
 *
 * This class defines operator<< for logging the contents of the queue. At the same time
 * we define a way to sort the received context in correct order. Meaning, that the request containing
 * the velocity requests are placed on first position with the highest priority.
 *
 * WARN:
 * We use this type to define ts::Queue::m_pqueue. Printing the contents
 * of this class must be handled with locked mutex.
 */
class my_queue
	: public std::priority_queue<std::string, std::vector<std::string>, std::greater<std::string>>
{
public:

	/**
	 * @brief Copies the contents of implementing structure to the std::ostream.
	 *
	 * The implementing structure is in our case std::vector. This allows us to copy
	 * all the contents to the std::ostream.
	 *
	 * @param os Output stream to which will the context be printed.
	 * @param queue Queue to be printed.
	 */
	friend std::ostream &operator<<(std::ostream &os, const my_queue &queue);
};

std::ostream &operator<<(std::ostream &os, const my_queue &queue);

/**
 * @class Queue
 * @brief Thread safe dynamic priority queue storing json strings
 * that are to be sent to the desired IP address. The preferred strings
 * are those that are setting the robot's left and right wheel speed.
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
	std::string pop();

	/**
	 * @brief Returns the copy of the top element in the priority queue.
	 */
	std::string peek();

	/**
	 * @brief Add a new element to the priority queue.
	 *
	 * Inserts the element to the queue. If the inserted item
	 * is sending request for setting the left and right wheel speed
	 * prioritize this request.
	 *
	 * @param item to be added to the queue.
	 */
	void push(const std::string &item);

private:
	/// Priority queue containing the requests to be send.
	my_queue m_pqueue;

	/// The queue mutex.
	std::mutex m_qMutex;

	/// Condition variable ensuring the pop method will wait for an element if the queue is empty.
	std::condition_variable m_cvPush;

	// The name of the queue used for debugging.
	const std::string m_queueName;
};

} // thread safe namespace

