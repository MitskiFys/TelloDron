#include "GamepadControl.h"
#include <iostream>
Gamepad::Gamepad(const std::string& input_device) :
	m_input_device(input_device),
	m_workThread(&Gamepad::ProcessEvents, this)
{
	m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
}

void Gamepad::ProcessEvents()
{
	js_event event;
	while (m_threadListenState)
	{
		while (read(m_fd, &event, sizeof(event)) != -1)
		{
			event.type &= ~JS_EVENT_INIT;
			if (event.type == JS_EVENT_BUTTON)
			{
				std::cout << "button" <<std::endl;
				ProcessButtonEvent(event.number, event.value);
				fireStateChanged(m_gamepadState);
			}
			else if (event.type == JS_EVENT_AXIS)
			{
				ProcessAxisEvent(event.number, normXYZValue(event.value));
				fireStateChanged(m_gamepadState);
			}
		}
		fireStateChanged(m_gamepadState);
		usleep(2E4);
	}
}

void Gamepad::ProcessButtonEvent(uint8_t number, const bool value)
{
	switch (number)
	{
	case DS4_BUTTONS::TRIANGLE:
	{
		m_gamepadState.buttonState.Triangle = value;
		break;
	}
	case DS4_BUTTONS::CROSS:
	{
		m_gamepadState.buttonState.Cross = value;
		break;
	}
	case DS4_BUTTONS::CIRCLE:
	{
		m_gamepadState.buttonState.Circle = value;
		break;
	}
	case DS4_BUTTONS::L1:
	{
		m_gamepadState.buttonState.L1 = value;
		break;
	}
	case DS4_BUTTONS::R1:
	{
		m_gamepadState.buttonState.R1 = value;
		break;
	}
	case DS4_BUTTONS::L2:
	{
		m_gamepadState.buttonState.L2 = value;
		break;
	}
	case DS4_BUTTONS::R2:
	{
		m_gamepadState.buttonState.R2 = value;
		break;
	}
	}
}

void Gamepad::ProcessAxisEvent(const uint8_t number, const int16_t value)
{
	switch (number)
	{
	case DS4_AXIS::L3_X:
	{
		m_gamepadState.axisState.L_X = value;
		break;
	}
	case DS4_AXIS::L3_Y:
	{
		m_gamepadState.axisState.L_Y = value;
		break;
	}
	case DS4_AXIS::R3_X:
	{
		m_gamepadState.axisState.R_X = value;
		break;
	}
	case DS4_AXIS::R3_Y:
	{
		m_gamepadState.axisState.R_Y = value;
		break;
	}
	}
}

int8_t Gamepad::normXYZValue(const int16_t value)
{
	return static_cast<int>(round((value / float(MAX_AXIS_VALUE)) * 100));
}
