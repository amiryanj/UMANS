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

#ifndef LIB_PLEDESTRIANS_H
#define LIB_PLEDESTRIANS_H

#include <core/costFunction.h>

/// @ingroup costfunctions
/// <summary>An implementation of the full navigation fuction proposed by Guy et al.,
/// in the paper "PLEdestrians: A least-effort approach to crowd simulation" (2010).</summary>
/// <remarks>
/// ### Notes: 
/// <list>
/// <item>This cost function already includes a goal-reaching component.</item>
/// <item>The original paper presents a closed-form solution for finding the optimal velocity.
/// Our implementation (currently) does not contain this closed-form solution.
/// To obtain an approximation of the original method, use this cost function in a sampling-based policy.</item>
/// <item>The <tt>%PLEdestrians.xml</tt> policy in our example files gives a good approximation of the original algorithm.</item>
/// </list>
/// 
/// ### Name in XML files:
/// <tt>%PLEdestrians</tt>
///
/// ### Parameters: 
/// <list>
/// <item>w_a, w_b (float): two weights for the main components of the cost function.</item>
/// <item>t_min, t_max (float): two time thresholds.</item>
/// </list>
/// </remarks>
class PLEdestrians : public CostFunction
{
private:
	float w_a = 2.23f; // weight in the cost function
	float w_b = 1.26f;
	float t_min = 0.5f;
	float t_max = 3;

public:
	PLEdestrians() : CostFunction() {}
	virtual ~PLEdestrians() {}
	const static std::string GetName() { return "PLEdestrians"; }

	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;

	void parseParameters(const CostFunctionParameters & params) override;
};

#endif //LIB_PLEDESTRIANS_H
