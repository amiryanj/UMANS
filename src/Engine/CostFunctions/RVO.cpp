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

#include <CostFunctions/RVO.h>
#include <core/worldBase.h>
#include <core/agent.h>

using namespace std;

float RVO::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{ 
	// disallow high speeds
	if (velocity.magnitude() > agent->getMaximumSpeed())
		return MaxFloat;
	
	const float radius = agent->getRadius();
	const Vector2D& position = agent->getPosition();
	const Vector2D& RVOVelocity = 2 * velocity - agent->getVelocity();
	
	const Vector2D& vDiff = agent->getPreferredVelocity() - velocity;

	// compute the smallest time to collision among all neighboring agents
	float minTTC = ComputeTimeToFirstCollision(position, RVOVelocity, radius, agent->getNeighbors(), range_, true);

	return w / minTTC + vDiff.magnitude();
}

void RVO::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("w", w);
}