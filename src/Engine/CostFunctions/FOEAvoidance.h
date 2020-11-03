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

#ifndef LIB_FOE_AVOIDANCE_H
#define LIB_FOE_AVOIDANCE_H

#include <core/costFunction.h>

/// @ingroup costfunctions
/// <summary>An implementation of the collision-avoidance cost function proposed by Lopez et al.,
/// in the paper "Character navigation in dynamic environments based on optical flow" (2019).</summary>
/// <remarks>This cost function attempts to achieve collision avoidance based on a neighbor's focus of expansion (FOE), 
/// which is a point in an agent's view indicating the neighbor's relative motion.
///
/// The gradient of this cost function has been explicitly implemented.
/// Thus, this cost function is suitable for use in a gradient-based policy.
/// 
/// ### Name in XML files:
/// <tt>%FOEAvoidance</tt>
/// ### Parameters:
/// None
/// </remarks>
class FOEAvoidance : public CostFunction
{
 public:
	FOEAvoidance() : CostFunction() {}
	virtual ~FOEAvoidance() {}
	const static std::string GetName() { return "FOEAvoidance"; }
  
	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;
	virtual Vector2D GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;
};

#endif //LIB_FOE_AVOIDANCE_H
