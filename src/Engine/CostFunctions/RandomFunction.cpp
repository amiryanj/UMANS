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

#include <CostFunctions/RandomFunction.h>
#include <core/agent.h>

float RandomFunction::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	return agent->ComputeRandomNumber(-1, 1);
}

Vector2D RandomFunction::GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	return Vector2D(agent->ComputeRandomNumber(-1, 1), agent->ComputeRandomNumber(-1, 1));
}
