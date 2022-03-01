#include "DronControl.h"
#include "nod/nod.hpp"
#include <iostream>

DronControl::DronControl():
	m_droneState(DroneState::LandOff),
	m_drone()
{
	m_drone.Bind();
	m_gamepad.fireStateChanged.connect([this](const GamepadState& state)
	{
		this->controlDrone(state);
	});
	m_drone.SendCommand("streamon");
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
	}

	//control
	if (m_droneState == DroneState::InAir)
	{
		std::string telloCommand = "rc ";
		telloCommand.append(std::to_string(state.axisState.R_X)).append(" ");
		telloCommand.append(std::to_string(state.axisState.R_Y * -1)).append(" ");
		telloCommand.append(std::to_string(state.axisState.L_Y * -1)).append(" ");
		telloCommand.append(std::to_string(state.axisState.L_X));
		m_drone.SendCommand(telloCommand);
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
