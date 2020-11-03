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

#ifndef LIB_POWER_LAW_H
#define LIB_POWER_LAW_H

#include <CostFunctions/ObjectInteractionForces.h>
#include <string>

/// @ingroup costfunctions
/// <summary>An implementation of the collision-avoidance force proposed by %Karamouzas et al.,
/// in the paper "Universal Power Law Governing Pedestrian Interactions" (2014).</summary>
/// <remarks>
/// This paper defines a force per neighboring agent, and thus the implementation inherits from ObjectInteractionForces.
///
/// ### Notes: 
/// <list>
/// <item>This cost function does *not* include a goal-reaching component. 
/// Thus, to bring agents to their goals, you need to combine this cost function with (for example) GoalReachingForce.</item>
/// <item>The <tt>%PowerLaw.xml</tt> policy in our example files reproduces the original algorithm.
/// using two cost functions: PowerLaw and GoalReachingForce.</item>
/// <item><strong>WARNING:</strong> This cost function does not yet support static obstacles, so any obstacles will be ignored for now.
/// This feature should be added in the future. Unfortunately, the original paper does not describe how obstacles are handled, 
/// and its original implementation cannot be freely used.</item>
/// </list>
/// 
/// ### Name in XML files:
/// <tt>%PowerLaw</tt>
///
/// ### Parameters:
/// <list>
/// <item>k (float): A scaling factor for the overall force.</item>
/// <item>tau0 (float): A time threshold, which could be interpreted as the "maximum time-to-collision of interest".</item>
/// </list>
/// </remarks>
class PowerLaw : public ObjectInteractionForces
{
private:
	float k = 1.5f;
	float tau0 = 3;

public:
	PowerLaw() : ObjectInteractionForces() {}
	virtual ~PowerLaw() {}
	const static std::string GetName() { return "PowerLaw"; }

	void parseParameters(const CostFunctionParameters & params) override;

protected:
	virtual Vector2D ComputeAgentInteractionForce(const Agent* agent, const PhantomAgent& other) const override;
	virtual Vector2D ComputeObstacleInteractionForce(const Agent* agent, const LineSegment2D& obstacle) const override;
};

#endif //LIB_POWER_LAW_H
