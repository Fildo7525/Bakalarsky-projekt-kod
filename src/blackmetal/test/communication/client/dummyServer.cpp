#include "dummyServer.hpp"

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <iostream>

#include <log.hpp>

INIT_MODULE(DummyServer);

Server::Server(int port)
	: m_port(port)
{
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = { 0 };
	const char* hello = "{\"status\":\"RECIEVE_OK\"}";
 
	// Creating socket file descriptor
	if ((m_serverFd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		FATAL("socket failed");
		exit(EXIT_FAILURE);
	}
	INFO("Socket created");
 
	// Forcefully attaching socket to the port 8080
	if (setsockopt(m_serverFd, SOL_SOCKET,
				   SO_REUSEADDR | SO_REUSEPORT, &opt,
				   sizeof(opt))) {
		FATAL("setsockopt");
		exit(EXIT_FAILURE);
	}

	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(m_port);
 
	// Forcefully attaching socket to the port 8080
	if (bind(m_serverFd, (struct sockaddr*)&address, sizeof(address)) < 0) {
		FATAL("bind failed");
		exit(EXIT_FAILURE);
	}
	INFO("Binding successfull");
	if (listen(m_serverFd, 3) < 0) {
		FATAL("listen");
		exit(EXIT_FAILURE);
	}
	INFO("Listeneing...");

	if ((m_socket = accept(m_serverFd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
		FATAL("accept failed");
		exit(EXIT_FAILURE);
	}
	INFO("New client accepted");

	while (true) {
		INFO("Reading...");

		clearBff(buffer, 1024);
		read(m_socket, buffer, 1024);
		printf("READ: %s\n", buffer);

		send(m_socket, hello, strlen(hello), 0);
		INFO("OK status send\n");

		auto tmp = handleMessage(std::string(buffer));
		if (tmp != "") {
			INFO("Sending the speed");
			send(m_socket, "{\"LeftWheelSpeed\"=150,\"RightWheelSpeed\"=150}", 44, 0);
			INFO("send: " << tmp);
		}
		INFO("");
	}
}

Server::~Server()
{
	INFO("Shutting down server");
	// closing the connected socket
	close(m_socket);
	// closing the listening socket
	shutdown(m_serverFd, SHUT_RDWR);
}

std::string Server::handleMessage(const std::string &msg)
{
	INFO("message: " << msg);
	if (msg.find("\"Command\":8") != std::string::npos || msg.find("\"Command\":6") != std::string::npos) {
		std::cout << "Returning speeds" << std::endl;
		// {"LeftWheelSpeed"=%ld "RightWheelSpeed"=%ld}
		return "{\"LeftWheelSpeed\"=150,\"RightWheelSpeed\"=150}";
	}
	std::cout << "Returning empty string" << std::endl;
	return "";
}

void Server::clearBff(char *buf, int size)
{
	for (int i = 0; i < size; i++) {
		buf[i] = 0;
	}
}

