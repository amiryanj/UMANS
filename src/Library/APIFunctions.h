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

#ifdef DLL_EXPORT
#ifdef WIN32
#define API_FUNCTION __declspec(dllexport) 
#else
#define API_FUNCTION 
#endif
#endif

extern "C"{
	/// <summary>A struct that describes the status of a single agent in the simulation.
	/// This struct is used for communication between the UMANS library and external applications.</summary>
	struct AgentData
	{
		/// The unique ID of the agent.
		int id;
		/// The x coordinate of the agent at the current time.
		float position_x;
		/// The y coordinate of the agent at the current time.
		float position_y;
		/// The x component of the agent's velocity at the current time.
		float velocity_x;
		/// The y component of the agent's velocity at the current time.
		float velocity_y;
		/// The x component of the agent's viewing direction at the current time.
		float viewingDirection_x;
		/// The y component of the agent's viewing direction at the current time.
		float viewingDirection_y;
	};

	/// <summary>Sets up a simulation based on a configuration file. 
	/// After this function call, the simulation will be ready for its first time step.</summary>
	/// <returns>true if the operation was successful; false otherwise, e.g. if the configuration file is invalid.</returns>
	API_FUNCTION  bool StartSimulation(const char* configFileName, int nrThreads);

	/// <summary>Gets the step size of the simulation, in seconds.</summary>
	/// <param ref="result_dt">[out] Will store the step size of the simulation.</param>
	/// <returns>true if the operation was successful; false otherwise, i.e. if the simulation has not been initialized (correctly) yet.</returns>
	API_FUNCTION bool GetSimulationTimeStep(float& result_dt);

	/// <summary>Performs the given number of simulation steps. 
	/// To obtain the resulting status of the simulation, use the GetAgentPositions() function.</summary>
	/// <param ref="nrSteps">The number of steps to perform.</param>
	/// <returns>true if the operation was successful; false otherwise, i.e. if the simulation has not been initialized (correctly) yet.</returns>
	API_FUNCTION bool DoSimulationSteps(int nrSteps);

	/// <summary>Gets the current status of all agents in the simulation.</summary>
	/// <param ref="result_agentData">[out] Will store a reference to an array of AgentData objects, 
	/// where each object describes the status of a single agent.</param>
	/// <param ref="result_nrAgents">[out] Will store the number of agents in the simulation, 
	/// i.e. the number of useful entries in the AgentData array.</param>
	/// <returns>true if the operation was successful; false otherwise, i.e. if the simulation has not been initialized (correctly) yet.</returns>
	API_FUNCTION bool GetAgentPositions(AgentData*& result_agentData, int& result_nrAgents);

	/// <summary>Overrides the position of some or all agents in the simulation. You can decide for yourself which agents are affected.</summary>
	/// <param ref="agentData">A reference to an array of AgentData objects, where each object describes the position of a single agent that you want to set.</param>
	/// <param ref="nrAgents">The number of entries in the AgentData array.</param>
	/// <returns>true if the operation was successful; false otherwise, i.e. if the simulation has not been initialized (correctly) yet.</returns>
	API_FUNCTION bool SetAgentPositions(AgentData* agentData, int nrAgents);

	/// <summary>Tries to add a new agent to the simulation.</summary>
	/// <param ref="x">The x-coordinate of the agent's starting position.</param>
	/// <param ref="y">The y-coordinate of the agent's starting position.</param>
	/// <param ref="radius">The agent's radius.</param>
	/// <param ref="prefSpeed">The agent's preferred speed.</param>
	/// <param ref="maxSpeed">The agent's maximum speed.</param>
	/// <param ref="maxAcceleration">The agent's maximum acceleration.</param>
	/// <param ref="policyID">The ID of the agent's policy to use for navigation.</param>
	/// <param ref="result_id">[out] Will store the ID of the agent that has been added.</param>
	/// <param ref="desiredID">[optional] A desired custom ID to use for the agent. This ID will be used if it is still available; 
	/// otherwise, the simulation will assign a different ID.</param>
	/// <returns>true if the operation was successful; false otherwise, 
	///  i.e. if the simulation has not been initialized (correctly) yet, or if the agent with the given ID does not exist.</returns>
	API_FUNCTION bool AddAgent(float x, float y, float radius, float prefSpeed, float maxSpeed, float maxAcceleration, int policyID, int& result_id, int desiredID = -1);

	/// <summary>Tries to remove a specific agent from the simulation.</summary>
	/// <param ref="id">The ID of the agent to remove.</param>
	/// <returns>true if the operation was successful; false otherwise, 
	///  i.e. if the simulation has not been initialized (correctly) yet, or if the agent with the given ID does not exist.</returns>
	API_FUNCTION bool RemoveAgent(int id);

	/// <summary>Changes the goal of a single agent in the simulation.</summary>
	/// <param ref="id">The ID of the agent to change.</param>
	/// <param ref="x">The x-coordinate of the agent's new goal.</param>
	/// <param ref="y">The y-coordinate of the agent's new goal.</param>
	/// <returns>true if the operation was successful; false otherwise, 
	///  i.e. if the simulation has not been initialized (correctly) yet, or if the agent with the given ID does not exist.</returns>
	API_FUNCTION bool SetAgentGoal(int id, float x, float y);

	/// <summary>Cleans up some objects related to the simulation. Call this method just before you finish using the UMANS library.</summary>
	/// <returns>true if the operation was successful; false otherwise, i.e. if the simulation has not been initialized (correctly) yet.</returns>
	API_FUNCTION bool CleanUp();
}