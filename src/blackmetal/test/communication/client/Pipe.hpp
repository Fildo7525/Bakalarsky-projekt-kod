#pragma once

#include <cstdio>
#include <cstring>
#include <errno.h>
#include <iostream>
#include <sched.h>
#include <sstream>
#include <unistd.h>

enum class PipeSide
	: int
{
	Parent,
	Child,
	None
};

template <typename T>
class Pipe
{
public:

	Pipe();
	Pipe(const Pipe &pipe) = delete;
	Pipe(Pipe &&pipe);

	bool close(PipeSide side) noexcept;
	size_t send(const T &data) noexcept;
	size_t receive(T &data);

private:
	const pid_t m_parent;
	int m_openedSide;
	int m_pipe[2];
};

template <typename T>
Pipe<T>::Pipe()
	: m_parent(getpid())
	, m_openedSide(int(PipeSide::None))
{
	if (pipe(m_pipe) == -1) {
		std::cerr << strerror(errno) << std::endl;
	}
}

template <typename T>
Pipe<T>::Pipe(Pipe &&pipe)
	: m_parent(std::move(pipe.m_parent))
	, m_openedSide(std::move(pipe.m_side))
	, m_pipe(std::move(pipe.m_pipe))
{
}

template <typename T>
bool Pipe<T>::close(PipeSide side) noexcept
{
	if (side == PipeSide::None) {
		return false;
	}

	::close(m_pipe[(side == PipeSide::Child ? 0 : 1)]);
	m_openedSide = (side == PipeSide::Child ? 1 : 0);
	return true;
}

template <typename T>
size_t Pipe<T>::send(const T &data) noexcept
{
	size_t ret = ::write(m_pipe[m_openedSide], (void*)(&data), sizeof(T));
	return ret;
}

template <typename T>
size_t Pipe<T>::receive(T &data)
{
	size_t ret = ::read(m_pipe[m_openedSide], (void *)(&data), sizeof(T));
	return ret;
}

void plog(const std::string &ss);
void chlog(const std::string &ss);

