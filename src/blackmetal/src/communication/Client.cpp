#include "Client.hpp"

#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <memory>
#include <cstdio>
#include <cstring>

INIT_MODULE(Client);

// wait for 200ms
#define WAIT_TIME 200'000

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
		WARN("The client is already connected to " << m_address << ':' << m_port);
		return;
	}

	DBG("The client is not initialized. Connecting to " << address << ':' << m_port);
	struct sockaddr_in serv_addr;
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

	int clientFD;
	if ((clientFD = connect(m_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
		FATAL("\nConnection Failed: " << strerror(clientFD));
		return;
	}

	m_connected = true;
	m_address = address;
	m_port = port;
	SUCCESS("Client connected to " << m_address << ':' << m_port);

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = WAIT_TIME;
	// Set timeout for receive to WAIT_TIME.
	setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
	// Set timeout for send to WAIT_TIME.
	setsockopt(m_socket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);
}

void Client::stop()
{
	// closing the connected socket
	m_connected = false;
	INFO("Closing connection with " << m_address << ':' <<  m_port);
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
		case bm::Command::EMPTY:
			return "bm::Command::EMPTY";
		case bm::Command::EMG_STOP:
			return "bm::Command::EMG_STOP";
		case bm::Command::NORMAL_STOP:
			return "bm::Command::NORMAL_STOP";
		case bm::Command::SET_LR_WHEEL_VELOCITY:
			return "bm::Command::SET_LR_WHEEL_VELOCITY";
		case bm::Command::SET_LR_WHEEL_POSITION:
			return "bm::Command::SET_LR_WHEEL_POSITION";
		case bm::Command::NONE_5:
			return "bm::Command::NONE_5";
		case bm::Command::GET_LR_WHEEL_VELOCITY:
			return "bm::Command::GET_LR_WHEEL_VELOCITY";
		case bm::Command::PREPARE_WHEEL_CONTROLLER:
			return "bm::Command::PREPARE_WHEEL_CONTROLLER";
		case bm::Command::GET_LR_WHEEL_POSITION:
			return "bm::Command::GET_LR_WHEEL_POSITION";
	}
	return "bm::Command Unknown";
}

bm::Status Client::sendRequest(bm::Command cmd, WheelValueT rightWheel, WheelValueT leftWheel)
{
	DBG("Composing command: " << stringifyCommand(cmd));
	// Example: "{\"UserID\":1,\"Command\":3,\"RightWheelSpeed\":0.1,\"LeftWheelSpeed\":0.1,\"RightWheelPosition\":0.1,\"LeftWheelPosition\":0.1}";
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(cmd));
	if (cmd == bm::Command::SET_LR_WHEEL_VELOCITY) {
		message += ",\"RightWheelSpeed\":" + std::to_string(std::get<double>(rightWheel)) +
					",\"LeftWheelSpeed\":" + std::to_string(std::get<double>(leftWheel));
	} else if (cmd == bm::Command::SET_LR_WHEEL_POSITION) {
		message += ",\"RightWheelPosition\":" + std::to_string(std::get<long>(rightWheel)) +
					",\"LeftWheelPosition\":" + std::to_string(std::get<long>(leftWheel));
	}
	message += "}";

	INFO("sending: " << message);

	bm::Status s = this->send(message);
	if (s != bm::Status::OK) {
		FATAL("Sending failed with status: " << stringifyStatus(s));
		return s;
	}
	s = receive(message);
	if (s != bm::Status::OK) {
		FATAL("Sending failed with status: " << stringifyStatus(s));
		return s;
	}
	return bm::Status::OK;
}

bm::Status Client::requestSpeed(double rightWheel, double leftWheel)
{
	return sendRequest(bm::Command::SET_LR_WHEEL_VELOCITY, rightWheel, leftWheel);
}

bm::Status Client::requestPosition(long rightWheel, long leftWheel)
{
	return sendRequest(bm::Command::SET_LR_WHEEL_POSITION, rightWheel, leftWheel);
}

bm::Status Client::send(const std::string &msg)
{
	int numberOfBytes;
	{
		std::lock_guard<std::mutex> lock(m_sendSynchronizer);
		numberOfBytes = ::send(m_socket, msg.c_str(), msg.size(), 0);
	}
	if (numberOfBytes < 0) {
		FATAL("Could not send the command to server");
		return bm::Status::SEND_ERROR;
	}
	else {
		DBG("The client sent " << msg);
	}
	return bm::Status::OK;
}

bm::Status Client::receive(std::string &msg)
{
	int numberOfBytes = 0;
	char buffer[1024] = { 0 };

	DBG("Receiving...");
	{
		std::lock_guard<std::mutex> lock(m_sendSynchronizer);
		numberOfBytes = ::read(m_socket, buffer, 1024);
	}
	if (numberOfBytes < 0) {
		FATAL("The data could not be received");
		return bm::Status::RECEIVE_ERROR;
	}
	msg.clear();
	msg = buffer;
	DBG("The server send " << numberOfBytes);
	return evalReturnState(msg);
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
	if (returnJson.find("RECIEVE_OK") == std::string::npos) {
		WARN("The robot buffer is full. The send data will not be used: " << returnJson);
		return bm::Status::FULL_BUFFER;
	}

	SUCCESS(returnJson);
	return bm::Status::OK;
}

