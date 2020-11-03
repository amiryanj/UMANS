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

#ifndef LIB_TTCADCA_H
#define LIB_TTCADCA_H

#include <core/costFunction.h>

/// @ingroup costfunctions
/// <summary>An implementation of the full navigation function proposed by Dutra et al.,
/// in the paper "Gradient-based steering for vision-based crowd simulation algorithms" (2017).</summary>
/// <remarks>This cost function is based on the the time and distance to closest approach (ttca, dca).
/// The original algorithm by Dutra et al. uses the gradient of this function (which has a closed form), and not the cost itself.
/// Thus, to reproduce the full navigation method, use a policy with gradient-based optimization.
///
/// ### Notes: 
/// <list>
/// <item>This cost function already includes a goal-reaching component.</item>
/// <item>The <tt>%Dutra.xml</tt> policy in our example files gives a good approximation of the original algorithm.</item>
/// <item>This algorithm originally simulates the agent's vision via rendering. We use a simple approximation of this.</item>
/// <item><strong>WARNING:</strong> Currently, TtcaDca::GetGradient() actually returns an approximation of the gradient (the standard one of CostFunction), 
/// and not the analytical one from the original paper. This is because the analytical gradient seems to yield incorrect behavior. 
/// At the moment, it is unclear whether this is a problem in our implementation or in the original equations.</item>
/// <item><strong>WARNING:</strong> This cost function does not yet support static obstacles, so any obstacles will be ignored for now.
/// This feature should be added in the future. To do this, we need to implement the concept of ttca/dca for line-segment obstacles, 
/// think of a good way to scale the cost by distance, and determine the gradient of this component.</item>
/// </list>
///
/// ### Name in XML files:
/// <tt>%TtcaDca</tt>
/// 
/// ### Parameters:
/// <list>
///	<item>sigmaTtca, sigmaDca (float): Scalars that define the steepness of two exponential functions in the "collision avoidance" part.</item>
///	<item>sigmaAngle_goal, sigmaSpeed_goal (float): Scalars that define the steepness of the two exponential functions in this cost function.</item>
/// </list>
/// </remarks>
class TtcaDca : public CostFunction
{
private:
	/*
	 * Parameters of the cost function
	*/
	float sigAngle_goal_ = 2;
	float sigSpeed_goal_ = 2;
	float sigTtca_ = 1;
	float sigDca_ = 0.3f;

	const float viewingAngleHalf_ = (float)(PI / 2.0);

    float costForTtcaDca(float ttca, float dca) const;

	float getMovementCost(const Vector2D& velocity, const Agent* agent) const;
	std::pair<float, float> getMovementCostGradient(const Vector2D& velocity, const Agent* agent) const;

public:
	TtcaDca() : CostFunction() {}
	virtual ~TtcaDca() {}
	const static std::string GetName() { return "TtcaDca"; }

	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const;
	virtual Vector2D GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const;

	void parseParameters(const CostFunctionParameters & params) override;

};

#endif //LIB_TTCADCA_H
