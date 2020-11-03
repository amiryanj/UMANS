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

#ifndef LIB_GOAL_REACHING_H
#define LIB_GOAL_REACHING_H

#include <CostFunctions/ForceBasedFunction.h>

/// @ingroup costfunctions
/// <summary>A force that attracts the agents towards its goal, as used by the Social Forces method (1995) and many other force-based methods.</summary>
/// <remarks>
/// ### Definition:
/// F = (agent.preferredVelocity - agent.currentVelocity) / max(dt, policy.relaxationTime).
/// We disallow relaxation times smaller than dt (the simulation time step) because this would yield undesired behavior.
/// ### Name in XML files:
/// <tt>%GoalReachingForce</tt></remarks>
class GoalReachingForce : public ForceBasedFunction
{
public:
	GoalReachingForce() : ForceBasedFunction() { range_ = 0; }
	virtual ~GoalReachingForce() {}
	const static std::string GetName() { return "GoalReachingForce"; }

protected:
	/// <summary>Computes a 2D force vector that steers the agent towards its preferred velocity.</summary>
	/// <param name="agent">The agent for which a force is requested.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	/// <returns>A force vector that steers the agent towards its preferred velocity.</returns>
	virtual Vector2D ComputeForce(Agent* agent, const WorldBase* world) const override;
};

#endif //LIB_GOAL_REACHING_H
