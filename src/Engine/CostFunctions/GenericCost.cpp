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

#include <CostFunctions/GenericCost.h>

float GenericCost::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	// Here you should compute a scalar that indicates the "cost" of using the given velocity.
	return 0;
}

Vector2D GenericCost::GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	// Here you should compute a 2D vector that is the gradient of the cost function at the given velocity.
	// If you do not know how to compute this, you can also leave out this method entirely, 
	// and then the parent class will automatically compute a numerical approximation of the gradient.
	// This is the same behavior as calling the parent version yourself:
	return CostFunction::GetGradient(velocity, agent, world);
}

void GenericCost::parseParameters(const CostFunctionParameters & params)
{
	// Here you may read custom parameters from "params", and call the parent version of parseParameters().
	// If you plan to do nothing except calling the parent version, you can also leave out this method entirely.
	CostFunction::parseParameters(params);
}
