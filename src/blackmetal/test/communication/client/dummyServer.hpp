#pragma once

#include <string>
#include <thread>

class Server
{
public:
	explicit Server(int port);
	~Server();

private:
	std::string handleMessage(const std::string &msg);
	void clearBff(char *buf, int size);
private:
	int m_port;
	int m_socket;
	int m_serverFd;
};

