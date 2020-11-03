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

#ifndef LIB_RVO_H
#define LIB_RVO_H

#include <core/costFunction.h>

/// @ingroup costfunctions
/// <summary>An implementation of the full navigation function proposed by van den Berg et al., 
/// in the paper "Reciprocal velocity obstacles for real-time multi-agent navigation" (2008).</summary>
/// <remarks>
/// ### Notes: 
/// <list>
/// <item>This cost function already includes a goal-reaching component.</item>
/// <item>Although the %RVO paper describes the navigation problem in a closed mathematical form, 
/// the authors eventually propose a sampling-based implementation. 
/// Thus, our implementation does not contain a closed-form solution for finding the optimal velocity.</item>
/// <item>This is **not** the %ORCA method (sometimes referred to as "RVO2") that uses linear programming.
/// If you want to use that, use the ORCA cost function instead.</item>
/// <item>The <tt>%RVO.xml</tt> policy in our example files reproduces the original algorithm.</item>
/// </list>
/// 
/// ### Name in XML files:
/// <tt>%RVO</tt>
///
/// ### Parameters: 
/// <list>
/// <item>w: A weight for one of the two main components of the cost function.</item>
/// </list>
/// </remarks>
class RVO : public CostFunction
{
private:
	float w = 1; // weight in the cost function

public:
	RVO() : CostFunction() {}
	virtual ~RVO() {}
	const static std::string GetName() { return "RVO"; }

	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;

	void parseParameters(const CostFunctionParameters & params) override;
};

#endif //LIB_RVO_H
