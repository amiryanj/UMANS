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

#ifndef LIB_MOUSSAID_H
#define LIB_MOUSSAID_H

#include <core/costFunction.h>

/// @ingroup costfunctions
/// <summary>An implementation of the full navigation function proposed by %Moussaid et al., 
/// in the paper "How simple rules determine pedestrian behavior and crowd disasters" (2011).</summary>
/// <remarks>
/// ### Notes: 
/// <list>
/// <item>This cost function already includes a goal-reaching component.</item>
/// <item>The <tt>%Moussaid.xml</tt> policy in our example files reproduces the original algorithm.</item>
/// </list>
/// 
/// ### Name in XML files:
/// <tt>%Moussaid</tt>
/// 
/// ### Parameters: 
/// <list>
/// <item>d_max: The maximum distance-to-collision to consider.</item>
/// </list>
/// </remarks>
class Moussaid : public CostFunction
{
private:
	float d_max = 8;

public:
	Moussaid() : CostFunction() {}
	virtual ~Moussaid() {}
	const static std::string GetName() { return "Moussaid"; }

	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;
	void parseParameters(const CostFunctionParameters & params) override;

private:
	float getDistanceToCollisionAtPreferredSpeed(const Vector2D& direction, const Agent* agent) const;
};

#endif //LIB_MOUSSAID_H
