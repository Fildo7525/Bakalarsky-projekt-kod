#include "Client.hpp"
#include "log.hpp"

#include <arpa/inet.h>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

Client::Client(int port, const std::string &address)
{
	struct sockaddr_in serv_addr;
	// std::string hello = "{\"UserID\":1,\"Command\":3,\"RightWheelSpeed\":1,\"LeftWheelSpeed\":1}";
	if ((m_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		FATAL("\n Socket creation error \n");
		return;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);

	// Convert IPv4 and IPv6 addresses from text to binary form
	if (inet_pton(AF_INET, address.c_str(), &serv_addr.sin_addr) <= 0) {
		FATAL("\nInvalid address Address: " << address << ", not supported \n");
		return;
	}

	if ((m_clientFD = connect(m_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
		FATAL("\nConnection Failed \n");
		return;
	}
}

Client::~Client()
{
	// closing the connected socket
	close(m_clientFD);
}

std::variant<bm::Status, std::string> Client::execute(bm::Command cmd, int rightWheel, int leftWheel)
{
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(cmd));
	message += ",\"RightWheelSpeed\":" + std::to_string(rightWheel) +
				",\"LeftWheelSpeed\":" + std::to_string(leftWheel) + "}";
	char buffer[1024] = { 0 };

	INFO("sending: " << message);

	int numberOfBytes = 0;
	if ((numberOfBytes = send(socketFD(), message.c_str(), message.size(), 0)) < 0) {
		FATAL("Could not send the command to server");
		return bm::Status::SEND_ERROR;
	}
	DBG("The server send" << numberOfBytes);

	if ((numberOfBytes = read(socketFD(), buffer, 1024)) < 0) {
		FATAL("The data could not be recieved");
		return bm::Status::RECIEVE_ERROR;
	}
	INFO("Recieved: " << buffer);
	DBG("The server send" << numberOfBytes);
	return buffer;
}

bm::Status Client::request(int rightWheel, int leftWheel)
{
	auto buffer = execute(bm::Command::SET_LR_WHEEL_VELOCITY, rightWheel, leftWheel);
	const bm::Status *tmp = std::get_if<bm::Status>(&buffer);
	if (!tmp) {
		return evalReturnState(std::get<std::string>(buffer));
	}
	return std::get<bm::Status>(buffer);
}

std::string Client::recieve()
{
	int numberOfBytes = 0;
	char buffer[1024] = { 0 };

	if ((numberOfBytes = read(socketFD(), buffer, 1024)) < 0) {
		FATAL("The data could not be recieved");
		return std::string();
	}
	INFO("Recieved: " << buffer);
	DBG("The server send" << numberOfBytes);
	return buffer;
}

int Client::socketFD()
{
	return m_socket;
}

static std::string retrieveAnswer(const std::string &msg)
{
	auto begin = msg.find(':');
	if (begin == std::string::npos) {
		begin = msg.find('=');
		if (begin == std::string::npos) {
			return "";
		}
	}

	auto end = msg.find('}')+1;
	return msg.substr(begin+1, end-1);
}

bm::Status Client::evalReturnState(const std::string &returnJson)
{
	std::string tmp = retrieveAnswer(returnJson);
	INFO(tmp);

	if (tmp == "RECIEVE_OK") {
		INFO("The execution ran correctly");
		return bm::Status::OK;
	} else {
		ERR("The rebot buffer is full. The send data will not be used.");
		return bm::Status::FULL_BUFFER;
	}
}

