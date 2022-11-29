#include "Client.hpp"

#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"

#include <chrono>
#include <thread>

#include <arpa/inet.h>
#include <asm-generic/errno.h>
#include <memory>
#include <cstdio>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

#define WAIT_TIME 200'000 // 200ms

INIT_MODULE(Client);

Client::Client(int port, const std::string &address)
	: m_queue()
{
	start(port, address);
	std::thread([this]{ workerThread(); }).detach();
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
		FATAL("Invalid address Address: " << address << ", not supported \n");
		return;
	}

	DBG("The address is valid and supported");

	if ((m_clientFD = connect(m_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
		FATAL("Connection Failed: " << strerror(m_clientFD));
		return;
	}
	m_connected = true;
	m_address = address;
	m_port = port;
	SUCCESS("Client connected to " << m_address << ':' << m_port);
	struct timeval tv;
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
	close(m_clientFD);
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
		case bm::Status::TIMEOUT_ERROR:
			return "bm::Status::TIMEOUT_ERROR";
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

void Client::execute(bm::Command cmd, double rightWheel, double leftWheel)
{
	DBG("Executing command: " << stringifyCommand(cmd) << " on " << m_address << ':' << m_port);
	// std::string hello = "{\"UserID\":1,\"Command\":3,\"RightWheelSpeed\":1,\"LeftWheelSpeed\":1}";
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(cmd));
	if (cmd == bm::Command::SET_LR_WHEEL_VELOCITY) {
		message += ",\"RightWheelSpeed\":" + std::to_string(rightWheel) +
					",\"LeftWheelSpeed\":" + std::to_string(leftWheel) + "}";
	} else {
		message += "}";
	}

	INFO("sending: " << message);

	m_queue.push(message);
}

void Client::request(double rightWheel, double leftWheel)
{
	execute(bm::Command::SET_LR_WHEEL_VELOCITY, rightWheel, leftWheel);
}

void Client::enqueue(const std::string &msg)
{
	m_queue.push(msg);
}

std::string Client::robotVelocity()
{
	return m_odometryMessages.pop();
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

bm::Status Client::send(const std::string &msg)
{
	int numberOfBytes;
	numberOfBytes = ::send(m_socket, msg.c_str(), msg.size(), 0);

	if (numberOfBytes < 0) {
		FATAL("Could not send the command to server");
		if (numberOfBytes == EAGAIN || numberOfBytes == EWOULDBLOCK) {
			return bm::Status::TIMEOUT_ERROR;
		}
		return bm::Status::SEND_ERROR;
	}
	else {
		SUCCESS("The client sent " << msg);
	}
	return bm::Status::OK;
	// return bm::Status::TIMEOUT_ERROR;
}

bm::Status Client::receive(std::string &msg)
{
	int numberOfBytes = 0;
	char buffer[1024] = { 0 };

	DBG("Receiving...");
	numberOfBytes = ::read(m_socket, buffer, 1024);

	if (numberOfBytes < 0) {
		FATAL("The data could not be received");
		if (numberOfBytes == EAGAIN || numberOfBytes == EWOULDBLOCK) {
			return bm::Status::TIMEOUT_ERROR;
		}
		return bm::Status::RECEIVE_ERROR;
	}
	msg.clear();
	msg = buffer;
	SUCCESS("The client received " << numberOfBytes);
	return evalReturnState(msg);
	// return bm::Status::TIMEOUT_ERROR;
}

void Client::workerThread()
{
	std::string message;
	bool failed = false;
	bool getSpeedCommand = false;
	while (m_connected) {
		if (!failed) {
			message = m_queue.pop();
		}
		if (message.find("Command\":6") != std::string::npos) {
			getSpeedCommand = true;
		}

		auto sendStatus = this->send(message);
		if (sendStatus != bm::Status::OK) {
			FATAL("Could not send: " << message << ". Trying again...");
			failed = true;
			continue;
		}

		auto receiveStatus = receive(message);
		if (receiveStatus != bm::Status::OK) {
			FATAL("The message could not be received with return state: " << stringifyStatus(receiveStatus));
			failed = true;
			continue;
		}

		if (getSpeedCommand) {
			std::string wheelSpeed;
			// We take a risk and do not check for an error. The connection is established at this point.
			// May be changed in the future.
			this->send("");
			receive(wheelSpeed);
			m_odometryMessages.push(wheelSpeed);
		}

		if (failed && sendStatus == bm::Status::OK && receiveStatus == bm::Status::OK) {
			failed = false;
		}
	}
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

