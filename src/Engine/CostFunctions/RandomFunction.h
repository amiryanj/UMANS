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

#ifndef LIB_RANDOM_FUNCTION_H
#define LIB_RANDOM_FUNCTION_H

#include <core/costFunction.h>

/// @ingroup costfunctions
/// <summary>A "fake" navigation function that produces random costs and gradients.</summary>
/// <remarks>This cost function can be used to add noise to an agent's navigation.
/// ### Name in XML files:
/// <tt>%RandomFunction</tt>
/// ### Parameters: 
/// None. You can use the common <tt>coeff</tt> parameter to scale the effect of this cost function.
/// </remarks>
class RandomFunction : public CostFunction
{
public:
	RandomFunction() : CostFunction() { range_ = 0; }
	virtual ~RandomFunction() {}
	const static std::string GetName() { return "RandomFunction"; }

	/// Computes a random cost between -1 and 1.
	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;

	/// Computes a random gradient with both the X and Y component between -1 and 1.
	virtual Vector2D GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;
};

#endif //LIB_RANDOM_FUNCTION_H
