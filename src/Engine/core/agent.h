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

#ifndef LIB_AGENT_H
#define LIB_AGENT_H

#include <iostream>
#include <tools/vector2D.h>
#include <core/policy.h>
#include <memory>
#include <random>
#include "../3rd-party/ORCA/ORCALine.h"

class WorldBase;

/// <summary>An agent in the simulation.</summary>
class Agent
{
public:
	/// <summary>A struct containing the settings of an agent that typically do not change over time.</summary>
	struct Settings
	{
		/// <summary>The radius of the agent's disk representation (in meters).</summary>
		float radius_ = 0.24f;
		/// <summary>The preferred walking speed of the agent (in meters per second).</summary>
		float preferred_speed_ = 1.4f;
		/// <summary>The maximum walking speed of the agent (in meters per second).</summary>
		float max_speed_ = 1.8f;
		/// <summary>The maximum acceleration of the agent (in meters per second squared).</summary>
		/// <remarks>To allow abrupt changes in velocity, use a high value, 
		/// combined with a low relaxation time in the agent's Policy.</remarks>
		float max_acceleration_ = 5.0f;

		/// <summary>A pointer to the policy that describes the agent's navigation behavior.</summary>
		Policy* policy_ = nullptr;

		/// <summary>Whether or not the agent should be removed when it has reached its goal.</summary>
		bool remove_at_goal_ = false;
	};

private:

	size_t id_;
	Agent::Settings settings_;
	std::string color_;

	Vector2D position_;
	Vector2D velocity_;
	Vector2D preferred_velocity_;
	Vector2D next_velocity_;
	Vector2D goal_;

	Vector2D velocity_without_forces_;
	Vector2D viewing_direction_;

	Vector2D contact_forces_;

	NeighborList neighbors_;

	// Private constructor; only the world should create agents
	Agent(size_t id, const Agent::Settings& settings);
	friend WorldBase;

	// Random-number generation
	std::default_random_engine RNGengine_;

	ORCALibrary::Solution orcaSolution_;

public:

#pragma region [Simulation-loop methods]
	/// @name Simulation-loop methods
	/// Public methods that an agent executes once in each frame of the simulation loop.
	/// @{

	/// <summary>Performs a nearest-neighbor query for this agent, using the search radius specified in its cost functions.</summary>
	/// <remarks>The search radius is the largest value of the "range" parameter among all cost functions in the agent's Policy.
	/// The result will be stored in a NeighborList object inside the agent. 
	/// You can obtain this result via the Agent::GetNeighbors() method.</remarks>
	/// <param name="world">A reference to the world in which the simulation takes place.</param>
	void ComputeNeighbors(WorldBase* world);

	/// <summary>Computes a preferred velocity for the agent.</summary>
	/// <remarks>Because this framework only considers local navigation, 
	/// the preferred velocity is always the vector that points straight towards the goal, 
	/// with a length equal to the agent's preferred speed.</remarks>
	void ComputePreferredVelocity();

	/// <summary>Uses this agent's Policy to compute a new velocity for the agent.</summary>
	/// <remarks>The result will be stored internally in the agent.</remarks>
	/// <param name="world">A reference to the world in which the simulation takes place.</param>
	void ComputeNextVelocity(WorldBase* world);

	/// <summary>Computes forces with neighboring agents that are currently colliding with this agent.</summary>
	/// <remarks>The result will be stored internally in the agent.</remarks>
	/// <param name="world">A reference to the world in which the simulation takes place.</param>
	void ComputeContactForces(WorldBase* world);

	/// <summary>Uses the last computed "next velocity" to update the agent's velocity and position.</summary>
	/// <param name="dt">The time that is passing in the current simulation frame.</param>
	void UpdateVelocityAndPosition(float dt);

	/// @}
#pragma endregion

#pragma region [Basic getters]
	/// @name Basic getters
	/// Methods that directly return a value stored in the agent.
	/// @{

	/// <summary>Returns the unique ID of this agent.</summary>
	size_t getID() const { return id_; }
	/// <summary>Returns the agent's current position.</summary>
	inline const Vector2D& getPosition() const { return position_; }
	/// <summary>Returns the agent's last used velocity.</summary>
	inline const Vector2D& getVelocity() const { return velocity_; }
	/// <summary>Returns the agent's last computed preferred velocity.</summary>
	inline const Vector2D& getPreferredVelocity() const { return preferred_velocity_; }
	/// <summary>Returns the radius of the agent's disk representation.</summary>
	inline float getRadius() const { return settings_.radius_; }
	/// <summary>Returns the agent's preferred walking speed.</summary>
	inline float getPreferredSpeed() const { return settings_.preferred_speed_; };
	/// <summary>Returns the agent's maximum walking speed.</summary>
	inline float getMaximumSpeed() const { return settings_.max_speed_; };
	/// <summary>Returns the agent's maximum acceleration.</summary>
	inline float getMaximumAcceleration() const { return settings_.max_acceleration_; };
	/// <summary>Returns a pointer to the Policy that describes the agent's navigation behavior.</summary>
	inline Policy* getPolicy() const { return settings_.policy_; }
	/// <summary>Returns whether or not the agent wants to be removed from the simulation when it reaches its goal.</summary>
	inline bool getRemoveAtGoal() const { return settings_.remove_at_goal_; }
	/// <summary>Returns the agent's goal position.</summary>
	inline const Vector2D& getGoal() const { return goal_; }
	/// <summary>Returns the agent's current viewing direction.</summary>
	inline const Vector2D& getViewingDirection() const { return viewing_direction_; }
	/// <summary>Returns the (most recently computed) list of neighbors for this agent.</summary>
	/// <returns>A non-mutable reference to the list of neighbors that this agent has last computed.</returns>
	inline const NeighborList& getNeighbors() const { return neighbors_; }

	/// @}
#pragma endregion

#pragma region [Advanced getters]
	/// @name Advanced getters
	/// Methods that compute and return a result based on the agent's internal state.
	/// @{

	/// <summary>Checks and returns whether the agent has reached its goal position.</summary>
	/// <returns>true if the agent's current position is sufficiently close to its goal; false otherwise.</returns>
	bool hasReachedGoal() const;

	/// @}
#pragma endregion

#pragma region [Basic setters]
	/// @name Basic setters
	/// Methods that directly change the agent's internal status.
	/// @{

	/// <summary>Overrides the position of this agent with the given value.</summary>
	/// <remarks>Note: This us not the usual way to change the agent's position. 
	/// Only use this if you wish to override the agent's standard motion mechanism.</remarks>
	/// <param name="position">The new position of the agent.</param>
	void setPosition(const Vector2D &position);

	/// <summary>Overrides the velocity and viewing direction of this agent with the given values, 
	/// for example computed by an external application.</summary>
	/// <remarks>Note: This us not the usual way to change the agent's velocity. 
	/// Only use this if you wish to override the agent's standard motion mechanism.</remarks>
	/// <param name="position">The new position of the agent.</param>
	void setVelocity_ExternalApplication(const Vector2D& velocity, const Vector2D& viewingDirection);

	/// <summary>Sets the goal position of this agent to the given value, and possibly updates the agent's viewing direction.</summary>
	/// <param name="goal">The new goal position of this agent.</param>
	void setGoal(const Vector2D &goal);

	/// @}
#pragma endregion

	/// <summary>Computes and returns a random floating-point number using this agent's random-number generator.</summary>
	/// <param name="min">A minimum value.</param>
	/// <param name="max">A maximum value.</param>
	/// <returns>A random number between min and max, obtained via uniform random sampling.</returns>
	float ComputeRandomNumber(float min, float max);

#pragma region [ORCA]

	inline const ORCALibrary::Solution& GetOrcaSolution() const { return orcaSolution_; }
	inline ORCALibrary::Solution& GetOrcaSolution() { return orcaSolution_; }

#pragma endregion

};

#endif //LIB_AGENT_H
