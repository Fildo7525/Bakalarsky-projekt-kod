#pragma once

#include <condition_variable>
#include <queue>
#include <string>
#include <mutex>
#include <vector>

namespace ts
{

class Queue {
	// Moved out of public interface to prevent races between this
	// and pop().
	bool empty() const;

public:
	Queue() = default;
	Queue(const Queue &) = delete;
	Queue& operator=(const Queue &) = delete;
 
	unsigned long size() const;
	std::string pop();
	void push(const std::string &item);

private:
	std::priority_queue<std::string, std::vector<std::string>, std::greater<std::string>> m_pqueue;
	// std::queue<std::string> m_queue;
	mutable std::mutex m_qMutex;
	mutable std::mutex m_communicationMutex;
	std::condition_variable m_cvPush;
};

} // thread safe

