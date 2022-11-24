#include "Client.hpp"

#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

INIT_MODULE(Client);

// wait for 200ms
#define WAIT_TIME 200'000

Client::Client(int port, const std::string &address)
	: m_queue(new ts::Queue("m_clientQueue"))
	, m_odometryMessages(new ts::Queue("m_odometryQueue"))
{
	start(port, address);
	std::thread([=] {
		sleep(1);
		workerThread();
	}).detach();
}

Client::~Client()
{
	stop();
}

void Client::start(int port, const std::string &address)
{
	if (m_connected) {
		WARN("The client is already connected to " << m_address << ':' << port);
		return;
	}

	DBG("The client is not initialized. Connecting to " << address << ':' << port);
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

bm::Status Client::sendRequest(bm::Command cmd, WheelValueT rightWheel, WheelValueT leftWheel)
{
	DBG("Composing command: " << bm::stringifyCommand(cmd));
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

	this->enqueue(message);

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

void Client::enqueue(const std::string &msg)
{
	m_queue->push(msg);
}

std::string Client::robotVelocity()
{
	DBG("Getting robot velocity");
	auto front = m_odometryMessages->pop();
	return front;
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

	DBG("The client sent " << msg);
	return bm::Status::OK;
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
	DBG("The client send " << numberOfBytes);
	return evalReturnState(msg);
}

void Client::workerThread()
{
	std::string message;
	bool getSpeedCommand = false;

	// Lambda function used for receiving messages from the server.
	auto _receive = [this] (std::string &message) -> bm::Status {
		auto receiveStatus = receive(message);
		if (receiveStatus != bm::Status::OK) {
			FATAL("The message could not be received with return state: " << stringifyStatus(receiveStatus));
		} else {
			message = "";
			DBG("The data were received");
		}
		return receiveStatus;
	};

	// Lambda function used for sending messages to the server.
	auto _send = [this] (const std::string &message) -> bm::Status {
		auto sendStatus = this->send(message);
		if (sendStatus != bm::Status::OK) {
			FATAL("Could not send: " << message << ". Trying again...");
		} else {
			DBG("The data were send");
		}
		return sendStatus;
	};

	while (m_connected) {
		message = m_queue->pop();

		if (message.find("Command\":6") != std::string::npos) {
			SUCCESS("sending: " << message);
			getSpeedCommand = true;
		} else if (message.find("Command") != std::string::npos) {
			INFO("sending: " << message);
		}

		_send(message);
		_receive(message);

		if (getSpeedCommand) {
			getSpeedCommand = false;
			std::string wheelSpeed;
			// We take a risk and do not check for an error. The connection is established at this point.
			// May be changed in the future.
			INFO("Waite for the velocity");
			receive(wheelSpeed);
			WARN("Pushing data " << wheelSpeed << " to m_odometryMessages");
			m_odometryMessages->push(wheelSpeed);
		}
	}
}

