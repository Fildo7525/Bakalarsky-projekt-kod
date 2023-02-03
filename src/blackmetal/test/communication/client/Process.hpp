#pragma once

#include "Pipe.hpp"

#include <cstring>
#include <functional>
#include <sched.h>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <utility>

/**
 * @brief Process class creates a new process and runs the requested callable object with specified arguments.
 */
template<typename ReturnT>
class Process
{
public:
	Process() = default;

	template<typename Callable, typename... Arguments>
	Process(Callable&& callable, Arguments&&... arguments);

	template<typename Callable, typename... Arguments>
	void run(Callable&& callable, Arguments&&... arguments);

	/**
	 * @brief Blocking function. The function tries to read data from pipe.
	 * The data are written there by the child process.
	 *
	 * @return Returned value is the output value of the function specified int he run function or constructor.
	 */
	ReturnT getResult();

	/**
	 * @brief Kills the child process.
	 */
	void kill();

private:
	/**
	 * @brief Sends data from the child process to the parent process using a pipe.
	 *
	 * @param data Data to be shared.
	 */
	void shareData(const ReturnT &data);

private:
	Pipe<ReturnT> m_pipe;
	/// Pipe sharing data of a specified type.
	/// Value returned from fork() function.
	pid_t m_childPID;
};

template<typename ReturnT>
template<typename Callable, typename... Arguments>
Process<ReturnT>::Process(Callable&& callable, Arguments&&... arguments)
	: m_pipe()
{
	this->run(std::forward<Callable>(callable), std::forward<Arguments>(arguments)...);
}

template<typename ReturnT>
template<typename Callable, typename... Arguments>
void Process<ReturnT>::run(Callable&& callable, Arguments&&... arguments)
{
	static_assert(std::is_invocable_v<Callable, Arguments...>, "The Callable is not invocable");

	m_childPID = fork();
	if (m_childPID == -1) {
		throw std::runtime_error("Child process could not be created");
	}

	if (m_childPID == 0) {
		m_pipe.close(PipeSide::Child);
		if constexpr(std::is_same<std::invoke_result_t<Callable, Arguments...>, void>::value) {
			std::invoke(std::forward<Callable>(callable), std::forward<Arguments>(arguments)...);
		} else {
			shareData(std::invoke(std::forward<Callable>(callable), std::forward<Arguments>(arguments)...));
		}
		exit(0);
	}
	m_pipe.close(PipeSide::Parent);
}

template<typename ReturnT>
ReturnT Process<ReturnT>::getResult()
{
	ReturnT data;
	m_pipe.receive(data);
	return data;
}

template<typename ReturnT>
void Process<ReturnT>::shareData(const ReturnT &data)
{
	m_pipe.send(data);
}

template<typename ReturnT>
void Process<ReturnT>::kill()
{
	auto ret = ::kill(m_childPID, SIGTERM);
	if (ret < 0) {
		throw std::runtime_error(strerror(errno));
	}
}

