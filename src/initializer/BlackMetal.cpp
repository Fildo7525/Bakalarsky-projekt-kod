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

Speed bm::BlackMetal::execute(Command cmd, const double leftWheelVelocity, const double rightWheelVelocity)
{
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(cmd));
	message += ",\"RightWheelSpeed\":" + std::to_string(rightWheelVelocity) +
				",\"LeftWheelSpeed\":" + std::to_string(leftWheelVelocity) + "}";
	char buffer[1024] = { 0 };

	send(m_socket, message.c_str(), message.size(), 0);
	if (read(m_socket, buffer, 1024) < 0) {
		return {};
	}
	if (cmd == bm::Command::GET_LR_WHEEL_VELOCITY) {
		char buf[1024] = { 0 };
		if (read(m_socket, buf, 1024) < 0) {
			return {};
		}
		return parseData(buf);
	}
	return {};
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

Speed bm::BlackMetal::parseData(std::string jsonMessage)
{
	long lws, rws;

	auto lws_start = jsonMessage.find_first_of('=') + 1;
	auto lws_end = jsonMessage.find_first_of(' ');
	lws = std::stol(jsonMessage.substr(lws_start, lws_end));

	auto rws_start = jsonMessage.find_last_of('=') + 1;
	auto rws_end = jsonMessage.find_last_of('}');
	rws = std::stol(jsonMessage.substr(rws_start, rws_end));

	return {lws, -rws};
}

bm::BlackMetal::Status bm::BlackMetal::evalReturnState(const char *returnJson)
{
	std::string msg(returnJson);
	std::string tmp(msg.begin()+18,msg.begin()+27);
	// std::cout << tmp;

	if (tmp == "RECIEVE_OK") {
		return Status::OK;
	} else {
		return Status::FULL_BUFFER;
	}
}

