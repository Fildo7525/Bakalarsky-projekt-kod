#include "Client.hpp"
#include "log.hpp"

#include "stopwatch/Stopwatch.hpp"

#include <arpa/inet.h>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

INIT_MODULE(Client);

Client::Client(int port, const std::string &address)
{
	start(port, address);
}

Client::~Client()
{
	stop();
}

void Client::start(int port, const std::string &address)
{
	if (m_connected) {
		WARN("The client is already connected to " << m_address);
		return;
	}

	m_address = address;
	DBG("The client is not initialized. Connecting to " << m_address);
	struct sockaddr_in serv_addr;
	int clientFD;
	// std::string hello = "{\"UserID\":1,\"Command\":3,\"RightWheelSpeed\":1,\"LeftWheelSpeed\":1}";
	if ((m_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		FATAL("\n Socket creation error \n");
		return;
	}

	DBG("Socket created");

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);

	// Convert IPv4 and IPv6 addresses from text to binary form
	if (inet_pton(AF_INET, address.c_str(), &serv_addr.sin_addr) <= 0) {
		FATAL("\nInvalid address Address: " << address << ", not supported \n");
		return;
	}

	DBG("The address is valid and supported");

	if ((clientFD = connect(m_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
		FATAL("\nConnection Failed: " << strerror(clientFD));
		return;
	}
	INFO("Client successfully connected");
	m_connected = true;
}

void Client::stop()
{
	// closing the connected socket
	m_connected = false;
	close(m_socket);
}

std::string Client::stringifyStatus(const bm::Status status)
{
	switch (status) {
		case bm::Status::FULL_BUFFER:
			return "bm::Status::FULL_BUFFER";
		case bm::Status::RECEIVE_ERROR:
			return "bm::Status::RECEIVE_ERROR";
		case bm::Status::SEND_ERROR:
			return "bm::Status::SEND_ERROR";
		case bm::Status::OK:
			return "bm::Status::OK";
	}
	return "Unknown bm::Status";
}

std::string Client::stringifyCommand(const bm::Command command)
{
	switch (command) {
		case bm::Command::PREPARE_WHEEL_CONTROLLER:
			return "bm::Command::PREPARE_WHEEL_CONTROLLER";
		case bm::Command::GET_LR_WHEEL_VELOCITY:
			return "bm::Command::GET_LR_WHEEL_VELOCITY";
		case bm::Command::SET_LR_WHEEL_VELOCITY:
			return "bm::Command::SET_LR_WHEEL_VELOCITY";
		case bm::Command::EMG_STOP:
			return "bm::Command::EMG_STOP";
		case bm::Command::EMPTY:
			return "bm::Command::EMPTY";
		case bm::Command::NONE_4:
			return "bm::Command::NONE_4";
		case bm::Command::NONE_5:
			return "bm::Command::NONE_5";
		case bm::Command::NORMAL_STOP:
			return "bm::Command::NORMAL_STOP";
	}
	return "Unknown bm::Command";
}

std::variant<bm::Status, std::string> Client::execute(bm::Command cmd, int rightWheel, int leftWheel)
{
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(cmd));
	message += ",\"RightWheelSpeed\":" + std::to_string(rightWheel) +
				",\"LeftWheelSpeed\":" + std::to_string(leftWheel) + "}";

	INFO("sending: " << message);

	TIC;
	this->send(message);
	receive(message);
	TOC;
	DBG("The send and receive ran in " << Stopwatch::lastStoppedTime() << " micro seconds");

	return message;
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

bm::Status Client::send(const std::string &msg)
{
	int numberOfBytes = ::send(m_socket, msg.c_str(), msg.size(), 0);
	if (numberOfBytes < 0) {
		FATAL("Could not send the command to server");
		return bm::Status::SEND_ERROR;
	}
	DBG("The server send " << numberOfBytes << " bytes");
	return bm::Status::OK;
}

bm::Status Client::receive(std::string &msg)
{
	int numberOfBytes = 0;
	char buffer[1024] = { 0 };

	// INFO("Receiving...");
	if ((numberOfBytes = read(m_socket, buffer, 1024)) < 0) {
		FATAL("The data could not be received");
		return bm::Status::RECEIVE_ERROR;
	}
	msg.clear();
	msg = buffer;
	// INFO("Recieved: " << buffer);
	// DBG("The server send" << numberOfBytes);
	return bm::Status::OK;
}

std::string Client::address()
{
	return m_address;
}

int Client::socketFD()
{
	return m_socket;
}

bool Client::connected()
{
	return m_connected;
}

bm::Status Client::evalReturnState(const std::string &returnJson)
{
	(void) returnJson;
	return bm::Status::OK;
}

