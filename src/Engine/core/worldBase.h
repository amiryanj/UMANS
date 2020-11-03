/* UMANS: Unified Microscopic Agent Navigation Simulator
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettré
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

#ifndef LIB_WORLD_BASE_H
#define LIB_WORLD_BASE_H

#include <vector>
#include <memory>
#include <string>
#include <queue>
#include <limits>

#include <unordered_map>
typedef std::unordered_map<size_t, size_t> AgentIDMap;

#include <tools/Polygon2D.h>
#include <core/agent.h>
#include <core/AgentKDTree.h>

/// <summary>A reference to an agent in the simulation, possibly with a transposed position.
/// Nearest-neighbor queries in WorldBase return results of this type.</summary>
struct PhantomAgent
{
private:
	/// <summary>The amount by which the neighboring Agent's position should be offset, e.g. to account for wrap-around in a toric world.</summary>
	Vector2D positionOffset;
	Vector2D position;
	float distSqr;

public:
	/// <summary>A pointer to the original Agent in the simulation.</summary>
	const Agent* realAgent;

	/// <summary>Creates a PhantomAgent with the given details.</summary>
	/// <param name="agent">A pointer to the original Agent in the simulation.</param>
	/// <param name="queryPosition">The query position used for finding neighbors; used for precomputing the distance to this PhantomAgent.</param>
	/// <param name="posOffset">The amount by which the neighboring Agent's position should be offset, e.g. to account for wrap-around in a toric world.</param>
	PhantomAgent(const Agent* agent, const Vector2D& queryPosition, const Vector2D& posOffset)
		: realAgent(agent), positionOffset(posOffset) 
	{
		UpdatePositionAndDistance(queryPosition);
	}

	PhantomAgent() : realAgent(nullptr) {}

	/// <summary>Returns the (precomputed) position of this neighboring agent, translated by the offset of this PhantomAgent.</summary>
	inline Vector2D GetPosition() const { return position; }
	/// <summary>Returns the velocity of this neighboring agent.</summary>
	inline Vector2D GetVelocity() const { return realAgent->getVelocity(); }
	/// <summary>Returns the (precomputed) squared distance from this PhantomAgent to the query position that was used to find it.</summary>
	inline float GetDistanceSquared() const { return distSqr; }

	/// <summary>Pre-computes (or re-computes) the translated position of this PhantomAgent, as well as its distance to a query point.
	/// Use this method if you want to correct the PhantomAgent's data in a new frame, without having search the AgentKDTree again.</summary>
	inline void UpdatePositionAndDistance(const Vector2D& queryPosition)
	{
		position = realAgent->getPosition() + positionOffset;
		distSqr = distanceSquared(queryPosition, position);
	}

};

/// <summary>An abstract class describing a world in which a simulation can take place.</summary>
class WorldBase
{
public:

	/// <summary>An enum containing the types of world in which a simulation takes place.</summary>
	enum Type { UNKNOWN_WORLD_TYPE, INFINITE_WORLD, TORIC_WORLD };
	static Type StringToWorldType(const std::string& type);

private:

	typedef std::pair<Agent*, double> AgentTimePair; 
	struct AgentTimePairComparator
	{
		bool operator()(const AgentTimePair& a, const AgentTimePair& b)
		{
			return a.second > b.second;
		}
	};

	typedef std::priority_queue<AgentTimePair, std::vector<AgentTimePair>, AgentTimePairComparator> AgentQueue;
	
	/// <summary>A list of agents (sorted by time) that will be added to the simulation in the future.</summary>
	AgentQueue agentsToAdd;

	/// <summary>A mapping from agent IDs to positions in the agents_ list.</summary>
	/// <remarks>Because agents can be removed during the simulation, the ID of an agent is not necessarily the same 
	/// as its position in the list. This is why we need this extra administration.
	/// (We could also put all agents directly in a map, but then we could not loop over the agents in parallel with OpenMP.)</remarks>
	AgentIDMap agentPositionsInVector;

	/// <summary>The agent ID that will be used for the next agent that gets added.</summary>
	size_t nextUnusedAgentID;

protected:
	 
	/// <summary>The type of this world, e.g. infinite or toric.</summary>
	const Type type_;

	std::vector<Polygon2D> obstacles_;

	/// <summary>A list containing all agents that are currently in the crowd.</summary>
	std::vector<Agent*> agents_;

	/// <summary>A kd-tree of agent positions, used for nearest-neighbor queries.</summary>
	AgentKDTree* agentKDTree;

	/// <summary>The length (in seconds) of a simulation step.</summary>
	float delta_time_;

	/// <summary>The simulation time (in seconds) that has passed so far.</summary>
	double time_;
	
public:

#pragma region [Basic getters]
	/// @name Basic getters
	/// Methods that directly return a value stored in the world.
	/// @{

	/// <summary>Returns the list of agents. Use this if you want to retrieve information from all agents in an arbitrary order.</summary>
	/// <returns>A non-mutable reference to the list of Agent objects.</returns>
	inline const std::vector<Agent*>& GetAgents() const { return agents_; }

        /// <summary>Returns the number of agents.</summary>
        /// <returns>An integer value.</returns>
        inline const size_t GetNumberOfAgents() const { return agents_.size(); }

	/// <summary>Returns the list of obstacles.</summary>
	/// <returns>A non-mutable reference to the list of Polygon2D objects, each representing one obstacle in the world.</returns>
	inline const std::vector<Polygon2D>& GetObstacles() const { return obstacles_; }

        /// <summary>Returns the number of obstacles.</summary>
        /// <returns>An integer value.</returns>
        inline const size_t GetNumberOfObstacles() const { return obstacles_.size(); }

	/// <summary>Returns the current simulation time (in seconds).</summary>
	/// <returns>The time (in seconds) that has been simulated since the simulation started.</returns>
	inline double GetCurrentTime() const { return time_; }

	/// <summary>Returns the duration of a single simulation time step (in seconds), i.e. the time that is simulated in a single execution of DoStep().</summary>
	/// <returns>The durection of a single simulation time step (in seconds).</summary>
	inline float GetDeltaTime() const { return delta_time_; }

	/// <summary>Returns the type of this world, i.e. infinite or toric.</summary>
	/// <returns>The value of the Type enum describing the type of this world.</returns>
	inline Type GetType() { return type_; }

	/// @}
#pragma endregion

#pragma region [Basic setters]
	/// @name Basic setters
	/// Methods that directly change the world's internal status.
	/// @{

	/// <summary>Sets the number of parallel threads that this class may use for the simulation.</summary>
	/// <remarks>The given number is sent to OpenMP. Whether this number can be used depends on the numer of (virtual) cores of your machine.</remarks>
	/// <param name="nrThreads">The desired number of threads to use.</param>
	void SetNumberOfThreads(int nrThreads);

	/// <summary>Sets the length of simulation time steps.</summary>
	/// <param name="delta_time">The desired length (in seconds) of a single simulation frame.</param>
	inline void SetDeltaTime(float delta_time) { delta_time_ = delta_time; }

	/// @}
#pragma endregion

	/// <summary>Performs a single iteration of the simulation. 
	/// All agents compute their new desired velocity and then move forward.</summary>
	void DoStep();
	
	/// <summary>Computes and returns a list of all agents that are within a given radius of a given position.</summary>
	/// <remarks>Subclasses of WorldBase must implement this method, because the result may depend on special properties (e.g. the wrap-around effect in WorldToric).</remarks>
	virtual NeighborList ComputeNeighbors(const Vector2D& position, float search_radius, const Agent* queryingAgent) const = 0;

#pragma region [Finding, adding, and removing agents]
	/// @name Finding, adding, and removing agents
	/// Methods for finding, adding, and removing agents in the simulation.
	/// @{

	/// <summary>Finds and returns the agent with the given ID, if it exists.</summary>
	/// <param name="id">The ID of the agent to find.</param>
	/// <returns>A mutable pointer to the Agent stored under the given ID, or nullptr if this agent does not exist.</returns>
	Agent* GetAgent(size_t id);

	/// <summary>Finds and returns the agent with the given ID, if it exists.</summary>
	/// <param name="id">The ID of the agent to find.</param>
	/// <returns>A non-mutable pointer to the Agent stored under the given ID, or nullptr if this agent does not exist.</returns>
	const Agent* GetAgent_const(size_t id) const;

	/// <summary>Creates a new Agent object to be added to the simulation at the given time.</summary>
	/// <remarks>If the given time has already been reached, the agent will be added to the simulation immediately. 
	/// Otherwise, the agent will be scheduled for insertion at the given moment.</remarks>
	/// <param name="position">The start position of the agent.</param>
	/// <param name="settings">The simulation settings of the agent.</param>
	/// <param name="desiredID">(optional) The desired ID of the agent. 
	/// If it is not set, or if this ID is already taken, a suitable ID will be chosen automatically.</param>
	/// <param name="startTime">(optional) The simulation time at which the agent should be added. 
	/// If it is not set (or set to a value lower than the current simulation time), the agent will be added immediately.
	/// Otherwise, the agent will be scheduled for insertion at the given moment.</param>
	/// <returns>A pointer to the newly created Agent object. The agent may have a different ID than the desired one.
	/// Also, the WorldBase class manages the memory of agents, so you do not have to delete this object yourself.</returns>
	Agent* AddAgent(const Vector2D& position, const Agent::Settings& settings, size_t desiredID = std::numeric_limits<size_t>::max(), float startTime = 0);

	/// <summary>Tries to remove the agent with the given ID from the simulation.</summary>
	/// <param name="id">The ID of the agent to remove.</param>
	/// <returns>true if the agent was successfully removed; false otherwise, i.e. if the agent with the given ID does not exist.</returns>
	bool RemoveAgent(size_t id);

	/// @}
#pragma endregion

	/// <summary>Adds an obstacle with the given vertices to the world.</summary>
	/// <param name="points">A sequence of 2D points defining the obstacle's boundary vertices.</param>
	virtual void AddObstacle(const std::vector<Vector2D>& points);

	/// <summary>Cleans up this WorldBase object for removal.</summary>
	virtual ~WorldBase();

protected:

	/// <summary>Creates a WorldBase object of the given type.</summary>
	WorldBase(Type type);

	/// <summary>Computes a list of all agents that lie within a given radius of a given position.</summary>
	/// <param name="position">A query position.</param>
	/// <param name="search_radius">A query radius.</param>
	/// <param name="queryingAgent">A pointer to the Agent object performing the query. This agent will be excluded from the results.
	/// Use nullptr to not exclude any agents.</param>
	/// <param name="result">[out] Will store a list of pointers to agents queryingAgent lie within "search_radius" meters of "position", 
	/// excluding the agent denoted by "queryingAgent" (if it exists).</returns>
	void computeNeighboringAgents_Flat(const Vector2D& position, float search_radius, const Agent* queryingAgent, std::vector<const Agent*>& result) const;

	void computeNeighboringObstacles_Flat(const Vector2D& position, float search_radius, std::vector<LineSegment2D>& result) const;

	/// <summary>Subroutine of DoStep() that moves all agents forward using their last computed "new velocities".</summary>
	/// <remarks>Subclasses of WorldBase may override this method if they require special behavior (e.g. the wrap-around effect in WorldToric).</remarks>
	virtual void DoStep_MoveAllAgents();

private:
	/// Adds a (previously created) agent to the simulation.
	void addAgentToList(Agent* agent);

	/// Removes the agent at a given position in the list, 
	/// and does the necessary management to keep this list valid.
	void removeAgentAtListIndex(size_t index);
};

#endif //LIB_WORLD_BASE_H
