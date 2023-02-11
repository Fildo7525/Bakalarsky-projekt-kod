#include "Client.hpp"

#include "ReturnStatus.hpp"
#include "stopwatch/Stopwatch.hpp"
#include "log.hpp"

#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <memory>
#include <thread>

INIT_MODULE(Client, dbg_level::DBG);

// wait for 200ms
#define WAIT_TIME 200'000

Client::Client(int port, const std::string &address)
	: m_connected(false)
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
	// This may be the case when the connection fails.
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

bm::Status Client::send(const std::string &msg)
{
	int numberOfBytes;
	numberOfBytes = ::send(m_socket, msg.c_str(), msg.size(), 0);

	if (numberOfBytes < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			FATAL("TIMEOUT_ERROR: Could not send the command to server");
			return bm::Status::TIMEOUT_ERROR;
		}
		FATAL("SEND_ERRROR: Could not send the command to server");
		return bm::Status::SEND_ERROR;
	}

	INFO("The client sent " << msg);
	return bm::Status::OK;
}

bm::Status Client::receive(std::string &msg)
{
	int numberOfBytes = 0;
	char buffer[1024] = { 0 };

	DBG("Receiving...");
	numberOfBytes = ::read(m_socket, buffer, 1024);

	if (numberOfBytes < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			FATAL("TIMEOUT_ERROR: The data could not be received");
			return bm::Status::TIMEOUT_ERROR;
		}
		FATAL("RECEIVE_ERROR: The data could not be received");
		return bm::Status::RECEIVE_ERROR;
	}

	msg.clear();
	msg = buffer;

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
	if (returnJson.find("FULL_BUFFER") != std::string::npos) {
		WARN("The robot buffer is full. The send data will not be used: " << std::quoted(returnJson));
		return bm::Status::FULL_BUFFER;
	}
	else if (returnJson.find("LeftWheelSpeed") != std::string::npos) {
		DBG("Received data are meant for Odometry class");
		return bm::Status::ODOMETRY_SPEED_DATA;
	}

	return bm::Status::OK;
}

