#include "DronControl.h"
#include "nod/nod.hpp"
#include <iostream>

namespace
{
	const auto XROTATIONCORR = 0.5;
	const auto YROTATIONCORR = 0.8;
}

DronControl::DronControl():
	m_droneState(DroneState::LandOff),
	m_drone(),
	m_gamepad()
{
	m_drone.Bind();
	m_gamepad.fireStateChanged.connect([this](const GamepadState& state)
	{
		this->controlDrone(state);
	});
	m_drone.SendCommand("streamon");
}

void DronControl::setActivePoseTracking(const bool isActive)
{
	m_isActiveTrackingPose = isActive;
	if (!isActive)
	{
		m_Xcorrection = 0;
		m_Ycorrection = 0;
	}
}

void DronControl::setXCorrection(const int correction)
{
	if (m_isActiveTrackingPose && m_isActiveTracking)
	{
		m_Xcorrection = correction;
	}
}

void DronControl::setYCorrection(const int correction)
{
	if (m_isActiveTrackingPose && m_isActiveTracking)
	{
		m_Ycorrection = correction;
	}
}

void DronControl::updateDroneState()
{
	auto optState = m_drone.GetState();
	if (!optState.has_value())
	{
		return;
	}
	auto state = optState.value();//pitch:0;roll:1;yaw:0;vgx:0;vgy:0;vgz:0;templ:75;temph:77;tof:10;h:0;bat:37;baro:106.73;time:0;agx:12.00;agy:-28.00;agz:-997.00;
	size_t pos = 0;
	std::string delimeter = ";";
	std::string token;
	while ((pos = state.find(delimeter)) != std::string::npos)
	{
		token = state.substr(0, pos);
		if (token.find("bat") != std::string::npos)
		{
			m_batteryLvl = state.substr(4, pos - 4);
		}
		if (token.find("temph") != std::string::npos)
		{
			m_tempLvl = state.substr(6, pos - 6);
		}
		state.erase(0, pos + delimeter.length());
	}
}

std::string DronControl::getBatteryLevel()
{
	return m_batteryLvl;
}

std::string DronControl::getTemp()
{
	return m_tempLvl;
}

void DronControl::controlDrone(const GamepadState &state)
{
	//takeoff
	if (m_droneState == DroneState::LandOff || m_droneState == DroneState::Emergency)
	{
		if (state.buttonState.R2 && state.buttonState.Triangle)
		{
			std::cout << "takeOff" << std::endl;
			m_droneState = DroneState::InAir;
			m_drone.SendCommand("takeoff");
		}
	}

	//land
	if (m_droneState == DroneState::InAir)
	{
		if (state.buttonState.R2 && state.buttonState.Cross)
		{
			std::cout << "land" << std::endl;
			m_droneState = DroneState::LandOff;
			const auto landResult = m_drone.SendCommand("land");
			std::cout << landResult << std::endl;
		}
	}

	//emergency
	if (state.buttonState.R2 && state.buttonState.L2)
	{
		std::cout << "Emergency" << std::endl;
		m_droneState = DroneState::Emergency;
		m_drone.SendCommand("emergency");
		m_isActiveTracking = false;
	}

	//control
	if (m_droneState == DroneState::InAir)
	{
		std::string telloCommand = "rc ";

		auto lxValue = state.axisState.L_X;
		auto lyValue = state.axisState.L_Y * -1;

		if (m_isActiveTracking && m_isActiveTrackingPose)
		{
			lxValue += m_Xcorrection * XROTATIONCORR;
			lyValue -= m_Ycorrection * YROTATIONCORR;
		}

		telloCommand.append(std::to_string(state.axisState.R_X)).append(" ");
		telloCommand.append(std::to_string(state.axisState.R_Y * -1)).append(" ");
		telloCommand.append(std::to_string(lyValue)).append(" ");
		telloCommand.append(std::to_string(lxValue));
		m_drone.SendCommand(telloCommand);
	}

	//traking on/off
	if (m_droneState == DroneState::InAir)
	{
		if (state.buttonState.Circle && !m_prevCircleState)
		{
			m_isActiveTracking = !m_isActiveTracking;
			m_prevCircleState = !m_prevCircleState;
			fireTrackingStateChanged(m_isActiveTracking);
		}
		if (!state.buttonState.Circle && m_prevCircleState)
		{
			m_prevCircleState = !m_prevCircleState;
		}
	}

	//speed
	if (m_droneState == DroneState::InAir)
	{
		if (m_prevSpeedState)
		{
			//add speed
			if (!state.buttonState.L1 && state.buttonState.R1)
			{
				m_speed += m_speed < 100 ? 10 : 0;
				m_prevSpeedState = false;
			}

			//minus speed
			if (state.buttonState.L1 && !state.buttonState.R1)
			{
				m_speed -= m_speed > 10 ? 10 : 0;
				m_prevSpeedState = false;
			}
		}
		if (!state.buttonState.L1 && !state.buttonState.R1)
		{
			m_drone.SendCommand("speed " + std::to_string(m_speed));
			m_prevSpeedState = true;
		}
	}


}
