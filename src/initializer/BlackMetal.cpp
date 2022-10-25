#include "BlackMetal.hpp"
#include <iostream>
#include <memory>
#include <string>

using bm::BlackMetal;

bool bm::BlackMetal::m_initialized = false;
std::shared_ptr<BlackMetal> bm::BlackMetal::m_instance =
	std::shared_ptr<BlackMetal>(new BlackMetal);

std::shared_ptr<bm::BlackMetal> bm::BlackMetal::instance()
{
	return m_instance;
}

bm::BlackMetal::~BlackMetal()
{
	// closing the connected socket
	close(m_clientFD);
}

bm::BlackMetal::Status bm::BlackMetal::execute(Command cmd, const int leftWheelVelocity, const int rightWheelVelocity)
{
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(cmd));
	message += ",\"RightWheelSpeed\":" + std::to_string(rightWheelVelocity) +
				",\"LeftWheelSpeed\":" + std::to_string(leftWheelVelocity) + "}";
	char buffer[1024] = { 0 };

	send(m_socket, message.c_str(), message.size(), 0);
	if (read(m_socket, buffer, 1024) < 0) {
		return Status::RECIEVE_ERROR;
	}
	return evalReturnState(buffer);
}

bm::BlackMetal::BlackMetal()
{
	struct sockaddr_in serv_addr;
	// std::string hello = "{\"UserID\":1,\"Command\":3,\"RightWheelSpeed\":1,\"LeftWheelSpeed\":1}";
	if ((m_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Socket creation error \n");
		return;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary form
	if (inet_pton(AF_INET, "192.168.1.3", &serv_addr.sin_addr) <= 0) {
		printf("\nInvalid address/ Address not supported \n");
		return;
	}

	if ((m_clientFD = connect(m_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
		printf("\nConnection Failed \n");
		return;
	}
}

bm::BlackMetal::Status bm::BlackMetal::evalReturnState(const char *returnJson)
{
	std::string msg(returnJson);
	std::string tmp(msg.begin()+18,msg.begin()+27);
	std::cout << tmp;

	if (tmp == "RECIEVE_OK") {
		return Status::OK;
	} else {
		return Status::FULL_BUFFER;
	}
}

