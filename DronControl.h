#pragma once

#include "ctello.h"
#include "GamepadControl.h"
#include <iostream>
enum class DroneState
{
	LandOff,
	InAir,
	Emergency
};

class DronControl
{
public:
	DronControl();

private:
	ctello::Tello m_drone;
	Gamepad m_gamepad;
	DroneState m_droneState;
	uint8_t m_speed{10};
	bool m_prevSpeedState{false};
	void controlDrone(const GamepadState& state);
};
