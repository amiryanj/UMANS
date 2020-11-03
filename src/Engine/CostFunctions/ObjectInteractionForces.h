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

#ifndef LIB_OBJECT_INTERACTION_FORCES_H
#define LIB_OBJECT_INTERACTION_FORCES_H

#include <CostFunctions/ForceBasedFunction.h>

/// <summary>An abstract class for a force-based function that computes a force per neighboring agent/obstacle and sums them up.</summary>
/// <remarks>The specific force per neighbor needs to be filled in by a child class, in the method ComputeAgentInteractionForce().</remarks>
class ObjectInteractionForces : public ForceBasedFunction
{
protected:
	ObjectInteractionForces() : ForceBasedFunction() {}
	virtual ~ObjectInteractionForces() {}

protected:
	/// <summary>Computes and returns a 2D force vector that the agent experiences.</summary>
	/// <remarks>In the case of ObjectInteractionForces, 
	/// this method calls ComputeAgentInteractionForce() and ComputeObstacleInteractionForce() for each neighbor of the querying agent, 
	/// and returns the sum of all these interaction forces.</remarks>
	/// <param name="agent">The agent for which a force is requested.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	/// <returns>A 2D vector containing the sum of all agent-interaction forces, as computed by ComputeAgentInteractionForce().</returns>
	virtual Vector2D ComputeForce(Agent* agent, const WorldBase* world) const override;

	/// <summary>Computes a 2D force vector that a given agent experiences due to a neighboring agent.</summary>
	/// <remarks>Subclasses of ObjectInteractionForces must implement this function.</remarks>
	/// <param name="agent">The agent for which a force is requested.</param>
	/// <param name="other">The neighboring agent.</param>
	/// <returns>A 2D vector describing the force that 'other' applies to 'agent', according to a particular force model.</returns>
	virtual Vector2D ComputeAgentInteractionForce(const Agent* agent, const PhantomAgent& other) const = 0;

	/// <summary>Computes a 2D force vector that a given agent experiences due to a neighboring obstacle segment.</summary>
	/// <remarks>Subclasses of ObjectInteractionForces must implement this function.</remarks>
	/// <param name="agent">The agent for which a force is requested.</param>
	/// <param name="obstacle">The neighboring obstacle segment.</param>
	/// <returns>A 2D vector describing the force that 'obstacle' applies to 'agent', according to a particular force model.</returns>
	virtual Vector2D ComputeObstacleInteractionForce(const Agent* agent, const LineSegment2D& obstacle) const = 0;
};

#endif //LIB_OBJECT_INTERACTION_FORCES_H
