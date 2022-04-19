#pragma once

#include "ctello.h"
#include "GamepadControl.h"
#include <iostream>
#include <atomic>

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
	void setActivePoseTracking(const bool isActive);
	void setXCorrection(const int correction);
	void setYCorrection(const int correction);
	void updateDroneState();
	std::string getBatteryLevel();
	std::string getTemp();
	nod::signal<void (const bool state)> fireTrackingStateChanged;

private:
	ctello::Tello m_drone;
	Gamepad m_gamepad;
	DroneState m_droneState;
	uint8_t m_speed{10};
	bool m_prevSpeedState{false};
	bool m_isActiveTracking {false};
	bool m_prevCircleState {false};
	std::atomic<bool> m_isActiveTrackingPose {false};
	std::atomic<int> m_Xcorrection {0};
	std::atomic<int> m_Ycorrection {0};
	std::string m_batteryLvl;
	std::string m_tempLvl;
	void controlDrone(const GamepadState& state);
};
