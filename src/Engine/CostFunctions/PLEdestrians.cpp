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

#include <CostFunctions/PLEdestrians.h>
#include <core/worldBase.h>
#include <core/agent.h>

using namespace std;

float PLEdestrians::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	float ttc = ComputeTimeToFirstCollision(agent->getPosition(), velocity, agent->getRadius(), agent->getNeighbors(), range_, true);
	if (ttc < t_min)
		return MaxFloat;

	return t_max * (w_a + w_b * velocity.sqrMagnitude())
		+ 2 * (agent->getGoal() - agent->getPosition() - t_max * velocity).magnitude() * sqrt(w_a*w_b);
}

void PLEdestrians::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("w_a", w_a);
	params.ReadFloat("w_b", w_b);
	params.ReadFloat("t_min", t_min);
	params.ReadFloat("t_max", t_max);
}