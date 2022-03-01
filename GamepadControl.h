#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <linux/joystick.h>
#include <thread>
#include <optional>
#include <atomic>
#include "nod/nod.hpp"

static constexpr char INPUT_DEVICE_NAME[]{"/dev/input/js0"};
static constexpr int MAX_AXIS_VALUE{32767};
static constexpr int AXIS_SCALE{100};

struct GamepadButtons
{
	bool Cross {false};
	bool Circle {false};
	bool Triangle {false};
	bool Square {false};
	bool L1 {false};
	bool L2 {false};
	bool R1 {false};
	bool R2 {false};
};

struct GamepadAxis
{
	int16_t L_X{0};
	int16_t L_Y{0};
	int16_t R_X{0};
	int16_t R_Y{0};
};

struct GamepadState
{
	GamepadButtons buttonState;
	GamepadAxis axisState;
};

class Gamepad
{
public:

	Gamepad(const std::string& inputDevice = INPUT_DEVICE_NAME);
	nod::signal<void (const GamepadState& state)> fireStateChanged;
private:

	enum DS4_BUTTONS
	{
		SQUARE,
		CROSS,
		CIRCLE,
		TRIANGLE,
		L1,
		R1,
		L2,
		R2,
	};

	enum DS4_AXIS
	{
		L3_X = 0,
		L3_Y,//1
		R3_X,//2
		LB_P,//3
		RB_P,//4
		R3_Y,//5
	};
private:

	void ProcessEvents();
	void ProcessButtonEvent(uint8_t number, const bool value);
	void ProcessAxisEvent(const uint8_t number, const int16_t value);

	std::atomic<bool> m_threadListenState{true};
	std::string m_input_device;
	int m_fd;
	std::thread m_workThread;
	GamepadState m_gamepadState;

	int8_t normXYZValue(const int16_t value);
};
