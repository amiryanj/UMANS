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

#include <CostFunctions/Moussaid.h>
#include <core/worldBase.h>
#include <core/agent.h>

using namespace std;

float Moussaid::getDistanceToCollisionAtPreferredSpeed(const Vector2D& direction, const Agent* agent) const
{
	const float prefSpeed = agent->getPreferredSpeed();
	const Vector2D& velocityWithPrefSpeed = direction * prefSpeed;

	// compute the time to collision at this velocity
	float ttc = ComputeTimeToFirstCollision(agent->getPosition(), velocityWithPrefSpeed, agent->getRadius(), agent->getNeighbors(), range_, false);

	// convert to the distance to collision, clamped to a maximum distance
	float distanceToCollision = (ttc == MaxFloat ? MaxFloat : ttc * prefSpeed);
	if (distanceToCollision > d_max)
		distanceToCollision = d_max;

	return distanceToCollision;
}

float Moussaid::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const float speed = velocity.magnitude(); 
	const float prefSpeed = agent->getPreferredSpeed();

	// compute the distance to collision at maximum speed
	const auto& dir = velocity.getnormalized();
	float DC = getDistanceToCollisionAtPreferredSpeed(dir, agent);

	// compute the cost for this direction, assuming maximum speed
	float K = d_max * d_max + DC * DC - 2 * d_max * DC*cosAngle(dir, agent->getPreferredVelocity());
	K += 1;

	// compute the optimal speed for this direction
	float S = std::min(prefSpeed, DC / agent->getPolicy()->getRelaxationTime());

	// return a cost that penalizes the difference with the optimal speed
	return (float)(K * (1 + pow((S - speed) / S, 2.0)));
}

void Moussaid::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("d_max", d_max);
}