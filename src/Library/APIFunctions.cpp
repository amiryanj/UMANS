/* UMANS: Unified Microscopic Agent Navigation Simulator
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettr�
**
** This program is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program. If not, see <https://www.gnu.org/licenses/>.
**
** Contact: crowd_group@inria.fr
** Website: https://project.inria.fr/crowdscience/
** See the file AUTHORS.md for a list of all contributors.
*/

#define DLL_EXPORT

#include "APIFunctions.h"

#include <core/crowdSimulator.h>
#include <algorithm>
#include <omp.h>
#include <cstring>
#include <string>
#include <clocale>

extern "C"
{
	CrowdSimulator* cs;
	AgentData* agentData;
	size_t agentDataSize;

	void resizeAgentData(bool deleteOldData = true)
	{
		if (deleteOldData)
			delete[] agentData;
		agentDataSize = (size_t)pow(2, 1 + (int)ceil(log2(cs->GetWorld()->GetAgents().size())));
		agentData = new AgentData[agentDataSize];
	}

	API_FUNCTION bool StartSimulation(const char* configFileName, int numberOfThreads)
	{
    	std::setlocale(LC_NUMERIC, "en_US.UTF-8");

		omp_set_num_threads(numberOfThreads);

		// initialize a new crowd simulation; we'll fill it with the contents of the given config file
		cs = CrowdSimulator::FromConfigFile(configFileName);
		if (cs == nullptr)
		{
			return false;
		}

		cs->GetWorld()->SetNumberOfThreads(numberOfThreads);

		// prepare the agentData array
		resizeAgentData(false);

		return true;
	}

	API_FUNCTION bool GetSimulationTimeStep(float& result_dt)
	{
		if (cs == nullptr)
			return false;

		result_dt = cs->GetWorld()->GetDeltaTime();
		return true;
	}

	API_FUNCTION bool DoSimulationSteps(int nrSteps)
	{
		if (cs == nullptr)
			return false;

		cs->RunSimulationSteps(nrSteps);
		return true;
	}

	API_FUNCTION bool GetAgentPositions(AgentData*& result_agentData, int& result_nrAgents)
	{
		if (cs == nullptr)
			return false;

		const auto& agents = cs->GetWorld()->GetAgents();

		// check if the AgentData array is large enough; resize it if necessary
		if (agents.size() > agentDataSize || agents.size() * 4 < agentDataSize)
			resizeAgentData(true);

		// fill the AgentData array with the current agent data
#pragma omp parallel for
		for (int i = 0; i < (int)agents.size(); ++i)
		{
			agentData[i].id = (int)agents[i]->getID();
			agentData[i].position_x = agents[i]->getPosition().x;
			agentData[i].position_y = agents[i]->getPosition().y;
			agentData[i].velocity_x = agents[i]->getVelocity().x;
			agentData[i].velocity_y = agents[i]->getVelocity().y;
			agentData[i].viewingDirection_x = agents[i]->getViewingDirection().x;
			agentData[i].viewingDirection_y = agents[i]->getViewingDirection().y;
		}

		// store references to these results, for the client program to use
		result_agentData = agentData;
		result_nrAgents = (int)agents.size();

		return true;
	}

	API_FUNCTION bool SetAgentPositions(AgentData* agentData, int nrAgents)
	{
#pragma omp parallel for
		for (int i = 0; i < nrAgents; ++i)
		{
			// find the agent with the given ID
			auto agent = cs->GetWorld()->GetAgent(agentData[i].id);
			if (agent != nullptr)
			{
				Vector2D position(agentData[i].position_x, agentData[i].position_y);
				Vector2D velocity(agentData[i].velocity_x, agentData[i].velocity_y);
				Vector2D viewingDirection(agentData[i].viewingDirection_x, agentData[i].viewingDirection_y);

				// override the agent's position, velocity, and viewing direction
				agent->setPosition(position);
				agent->setVelocity_ExternalApplication(velocity, viewingDirection);
			}

		}
		return true;
	}

	API_FUNCTION bool AddAgent(float x, float y, float radius, float prefSpeed, float maxSpeed, float maxAcceleration, int policyID, int& result_id, int customID)
	{
		if (cs == nullptr)
			return false;

		// fill in the agent's settings
		Agent::Settings settings;
		settings.radius_ = radius;
		settings.preferred_speed_ = prefSpeed;
		settings.max_speed_ = maxSpeed;
		settings.max_acceleration_ = maxAcceleration;
		settings.policy_ = cs->GetPolicy(policyID);

		// try to add an agent
		auto agent = cs->GetWorld()->AddAgent(
			Vector2D(x, y), 
			settings,
			(customID >= 0 ? customID : std::numeric_limits<size_t>::max()));
		
		if (agent != nullptr)
		{
			// fill in the ID that the agent received
			result_id = (int)agent->getID();

			// set the agent's goal to its current position, so that it doesn't start moving
			agent->setGoal(agent->getPosition());
		}
			

		return (agent != nullptr);
	}

	API_FUNCTION bool RemoveAgent(int id)
	{
		if (cs == nullptr)
			return false;

		// try to remove the agent
		return cs->GetWorld()->RemoveAgent(id);
	}

	API_FUNCTION bool SetAgentGoal(int id, float x, float y)
	{
		if (cs == nullptr)
			return false; 
		
		// try to find the agent
		auto* agent = cs->GetWorld()->GetAgent(id);
		if (agent == nullptr)
			return false;
		
		agent->setGoal(Vector2D(x, y));
		return true;
	}

	API_FUNCTION bool CleanUp()
	{
		if (cs == nullptr)
			return false;

		delete cs;
		cs = nullptr;
		delete[] agentData;
		agentData = nullptr;

		return true;
	}        
}
