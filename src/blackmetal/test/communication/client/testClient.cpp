#include "dummyServer.hpp"
#include "log.hpp"
#include <Client.hpp>
#include "Process.hpp"
#include <gtest/gtest.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <thread>

#include <unistd.h>

INIT_MODULE(TestClient, dbg_level::DBG);

#define PORT 8080

using namespace std::chrono_literals;
std::string myIP;

TEST(ClientTest, connect) {

	Process<int> process([&] { Server s(PORT); });

	system("hostname -I > /tmp/ip");
	std::ifstream file("/tmp/ip");
	file >> myIP;
	file.close();
	system("rm -rf /tmp/ip");

	INFO("Your IPv4: " << myIP);

	Client c(PORT, myIP);
	auto connected = c.connected();

	process.kill();

	EXPECT_TRUE(connected);
}

TEST(ClientTest, sendRequest) {
	Process<int> process([&] { Server s(PORT); });
	Client client(PORT, myIP);

	client.sendRequest(bm::Command::SET_LR_WHEEL_VELOCITY, 0.1, 0.1);
	client.sendRequest(bm::Command::GET_LR_WHEEL_VELOCITY);
	auto velocity = client.robotVelocity();
	std::string received = "{\"LeftWheelSpeed\"=150,\"RightWheelSpeed\"=150}";
	auto eq = velocity == received;

	client.stop();
	process.kill();

	EXPECT_TRUE(eq);
}

