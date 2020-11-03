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

#ifndef LIB_SOCIAL_FORCES_H
#define LIB_SOCIAL_FORCES_H

#include <CostFunctions/ObjectInteractionForces.h>

/// @ingroup costfunctions
/// <summary>An implementation of the collision-avoidance force proposed by Helbing and Molnar,
/// in the paper "Social force model for pedestrian dynamics" (1995).</summary>
/// <remarks>
/// This paper defines a force per neighboring agent, and thus the implementation inherits from ObjectInteractionForces.
///
/// ### Notes:
/// <list>
/// <item>This cost function does *not* include a goal-reaching component. 
/// Thus, to bring agents to their goals, you need to combine this cost function with (for example) GoalReachingForce.</item>
/// <item>The <tt>%SocialForces.xml</tt> policy in our example files reproduces the original algorithm, 
/// using two cost functions: SocialForces and GoalReachingForce.</item>
///
/// ### Name in XML files:
/// <tt>%SocialForcesAvoidance</tt>
/// 
/// ### Parameters:
/// <list>
/// <item>dt (float): A time window parameter used in force calculations. It does not have a very intuitive meaning.</item>
/// <item>V0 (float): A scaling factor for all agent-avoidance forces.</item>
/// <item>sigma (float): Determines the steepness of the exponential function defining an agent-avoidance force.
/// Could be interpreted as the distance (in meters) that an agent wants to keep from another agent.</item>
/// <item>U0 (float): A scaling factor for all obstacle-avoidance forces.</item>
/// <item>R (float): Determines the steepness of the exponential function defining an obstacle-avoidance force. 
/// Could be interpreted as the distance (in meters) that an agent wants to keep from obstacles.</item>
/// </list>
/// </remarks>
class SocialForcesAvoidance : public ObjectInteractionForces
{
private:
	float dt = 1;
	float V0 = 2.1f;
	float sigma = 0.3f;
	float U0 = 10;
	float R = 0.2f;

public:
	SocialForcesAvoidance() : ObjectInteractionForces() {}
	virtual ~SocialForcesAvoidance() {}
	const static std::string GetName() { return "SocialForcesAvoidance"; }
	void parseParameters(const CostFunctionParameters & params) override;

protected:
	virtual Vector2D ComputeAgentInteractionForce(const Agent* agent, const PhantomAgent& other) const override;
	virtual Vector2D ComputeObstacleInteractionForce(const Agent* agent, const LineSegment2D& obstacle) const override;
};

#endif //LIB_SOCIAL_FORCES_H
