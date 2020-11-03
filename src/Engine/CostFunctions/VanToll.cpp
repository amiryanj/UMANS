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

#include <CostFunctions/VanToll.h>
#include <core/worldBase.h>
#include <core/agent.h>
#include "core/costFunctionFactory.h"

using namespace std;

float VanToll::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const Vector2D& currentPos = agent->getPosition();
	const Vector2D& currentVel = agent->getVelocity();
	const Vector2D& prefVel = agent->getPreferredVelocity();
	const float prefSpeed = prefVel.magnitude();
	const float speed = velocity.magnitude();

	// compute the time to collision
	const float candidateTTC = ComputeTimeToFirstCollision(currentPos, velocity, agent->getRadius(), agent->getNeighbors(), range_, false);

	// compute the distance to collision, bounded by the viewing distance
	float distanceToCollision = (candidateTTC == MaxFloat ? MaxFloat : candidateTTC * speed);
	if (distanceToCollision > max_distance)
		distanceToCollision = max_distance;

	const float distanceToCollisionFrac = (max_distance - distanceToCollision) / max_distance;

	const float deviationFromPreferredAngle = angle(velocity, prefVel);
	const float deviationFromPreferredSpeed = abs(speed - prefSpeed) / prefSpeed;
	const float deviationFromCurrentAngle = angle(velocity, currentVel);

	// The cost of this velocity is a weighted sum of various factors
	const float Weight_DistanceToCollision = 1;
	const float Weight_DeviationFromPreferredAngle = 1;
	const float Weight_DeviationFromPreferredSpeed = 1;
	const float Weight_DeviationFromCurrentAngle = 1;

	const float candidateCost =
		Weight_DistanceToCollision * (max_distance - distanceToCollision) +
		Weight_DeviationFromPreferredAngle * deviationFromPreferredAngle +
		Weight_DeviationFromPreferredSpeed * deviationFromPreferredSpeed +
		Weight_DeviationFromCurrentAngle * deviationFromCurrentAngle;

	return candidateCost;
}

void VanToll::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("max_distance", max_distance);
}