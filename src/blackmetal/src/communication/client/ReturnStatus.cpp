#include "ReturnStatus.hpp"

std::string bm::stringifyStatus(const bm::Status status)
{
	switch (status) {
		case bm::Status::TIMEOUT_ERROR:
			return "bm::Status::TIMEOUT_ERROR";
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

std::string bm::stringifyCommand(const bm::Command command)
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

