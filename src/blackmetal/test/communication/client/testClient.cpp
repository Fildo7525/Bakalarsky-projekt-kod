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

// These tests sometimes work and sometimes don't.
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

	system("hostname -I > /tmp/ip");
	std::ifstream file("/tmp/ip");
	file >> myIP;
	file.close();
	system("rm -rf /tmp/ip");

	Client client(PORT, myIP, 200'000);

	client.send("");
	std::string velocity;
	auto status = client.receive(velocity);
	std::string received = "{\"status\":\"RECIEVE_OK\"}";
	auto eq = velocity == received;

	client.stop();
	process.kill();

	ASSERT_EQ(status, bm::Status::OK);
	EXPECT_TRUE(eq);
}

