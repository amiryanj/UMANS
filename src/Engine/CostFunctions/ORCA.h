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

#ifndef LIB_ORCA_H
#define LIB_ORCA_H

#include <core/costFunction.h>
#include "../3rd-party/ORCA/ORCALine.h"

/// @ingroup costfunctions
/// <summary>The cost function of the %ORCA collision-avoidance model proposed by van den Berg et al.,
/// in the paper "Reciprocal n-body collision avoidance" (2011).</summary>
/// <remarks>
/// ### Notes: 
/// <list>
/// <item>This cost function already includes a goal-reaching component.</item>
/// <item>The original paper describes a cost function, but also a closed-form solution for finding the optimal velocity.
/// Our implementation contains both options, using the original source code provided by the paper's authors.</item>
/// <item>The <tt>%ORCA.xml</tt> policy in our example files reproduces the original algorithm.</item>
/// <item><strong>WARNING:</strong> This cost function does not yet support static obstacles, so any obstacles will be ignored for now.
/// This feature should be added in the future. The original ORCA code uses a different obstacle representation than we do, 
/// which makes this integration a bit difficult.</item>
/// </list>
/// 
/// ### Name in XML files:
/// <tt>%ORCA</tt>
///
/// ### Parameters: 
/// <list>
/// <item>timeHorizon (float): The time (in seconds) over which velocity obstacles are defined.</item>
/// </list>
/// </remarks>
class ORCA : public CostFunction
{
private:
	float timeHorizon = 5;

public:
	ORCA() : CostFunction() {}
	virtual ~ORCA() {}
	const static std::string GetName() { return "ORCA"; }

	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;
	virtual Vector2D GetGlobalMinimum(Agent* agent, const WorldBase* world) const override;
	void parseParameters(const CostFunctionParameters & params) override;

private:
	const ORCALibrary::Solution& GetOrcaSolutionForAgent(Agent* agent, const WorldBase* world) const;
};

#endif //LIB_ORCA_H
