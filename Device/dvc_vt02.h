#pragma once

#include <cstdint>

#include "bsp_uart_port.h"
#include "../communication_topic/uart_topics.hpp"

struct VT02RecviedRawData {
	struct {
		uint8_t start_of_frame;
		uint16_t data_length;
		uint8_t seq;
		uint8_t crc_8;
	} frame_header;

	uint16_t cmd_id;

	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t l;
		uint8_t r;
	} mouse;

	union {
		uint16_t key_code;
		struct {
			uint16_t W : 1;
			uint16_t S : 1;
			uint16_t A : 1;
			uint16_t D : 1;
			uint16_t SHIFT : 1;
			uint16_t CTRL : 1;
			uint16_t Q : 1;
			uint16_t E : 1;
			uint16_t R : 1;
			uint16_t F : 1;
			uint16_t G : 1;
			uint16_t Z : 1;
			uint16_t X : 1;
			uint16_t C : 1;
			uint16_t V : 1;
			uint16_t B : 1;
		} bit;
	} keyboard;

	int16_t pulley_wheel;
	uint16_t crc16;
} __attribute__((packed));

class VT02 {
public:
	static VT02& Instance() {
		static VT02 instance;
		return instance;
	}

	void Init(BspUartHandle uart, orb::UartPort vt02_port);
	void RxCpltCallback(uint8_t* buffer, uint16_t length);

private:
	BspUartHandle uart_ = nullptr;
	orb::UartPort vt02_port_ = orb::UartPort::U1;
	bool started_ = false;

	const float kMouseSensitivityX = 30.0f;
	const float kMouseSensitivityY = -8.0f;

	VT02RecviedRawData pre_uart_rx_data_{};
};


