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

#include <CostFunctions/Karamouzas.h>
#include <core/worldBase.h>
#include <core/agent.h>

using namespace std;

float Karamouzas::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const auto& prefVelocity = agent->getPreferredVelocity();
	const auto& currentVelocity = agent->getVelocity();
	const auto& speed = velocity.magnitude();
	const float maxSpeed = agent->getMaximumSpeed();
	const auto& neighbors = agent->getNeighbors();

	float TTC_preferred = ComputeTimeToFirstCollision(agent->getPosition(), prefVelocity, agent->getRadius(), neighbors, range_, true);
	
	// This collision-avoidance method has a dynamic angular range, so ignore velocities that are outside it
	if (angle(velocity, prefVelocity) > getMaxDeviationAngle(agent, TTC_preferred))
		return MaxFloat;

	// same for speed
	if (speed < getMinSpeed(agent, TTC_preferred) || speed > getMaxSpeed(agent, TTC_preferred))
		return MaxFloat;
	
	// compute time to collision at this candidate velocity
	float TTC = ComputeTimeToFirstCollision(agent->getPosition(), velocity, agent->getRadius(), neighbors, range_, true);

	// the cost is a weighted sum of factors:

	float A = alpha * (1 - cosAngle(velocity, prefVelocity)/2.0f);
	float B = beta * abs(speed - currentVelocity.magnitude()) / maxSpeed;
	float C = gamma * (velocity - prefVelocity).magnitude() / (2 * maxSpeed);
	float D = delta * std::max(0.0f, t_max - TTC) / t_max;

	return A + B + C + D;
}

float Karamouzas::getMinSpeed(const Agent* agent, const float ttc) const
{
	float prefSpeed = agent->getPreferredSpeed();
	float maxSpeed = agent->getMaximumSpeed();
	float deltaSpeed = abs(maxSpeed - prefSpeed);

	const float epsilon = 0.05f; // to prevent imprecision issues;

	// no collision: always move at the preferred speed
	if (ttc <= 0 || ttc >= tc_max)
		return prefSpeed - epsilon;

	// collision very nearby: allow all speeds
	if (ttc > 0 && ttc < tc_min)
		return 0;

	// something in-between: allow a range of speeds around prefSpeed
	return prefSpeed - deltaSpeed - epsilon;
}

float Karamouzas::getMaxSpeed(const Agent* agent, const float ttc) const
{
	float prefSpeed = agent->getPreferredSpeed();
	float maxSpeed = agent->getMaximumSpeed();
	float deltaSpeed = abs(maxSpeed - prefSpeed);

	const float epsilon = 0.05f; // to prevent imprecision issues;

	// no collision: always move at the preferred speed
	if (ttc <= 0 || ttc >= tc_max)
		return prefSpeed + epsilon; 
	
	// collision very nearby: allow all speeds
	if (ttc > 0 && ttc < tc_min)
		return maxSpeed + epsilon;

	// something in-between: allow a range of speeds around prefSpeed
	return prefSpeed + deltaSpeed + epsilon;
}

float Karamouzas::getMaxDeviationAngle(const Agent* agent, const float ttc) const
{
	if (ttc <= 0 || ttc >= tc_max)
		return d_min;

	if (ttc < tc_min)
		// quadratic decrease: from d_max at TTC = 0, to d_mid at TTC = tc_min
		return d_mid + pow((tc_min - ttc)/tc_min, 2) * (d_max - d_mid);

	if (ttc < tc_mid)
		return d_mid;

	// ttc < tc_max: linear decrease from d_mid at tc_mid, to d_min at tc_max
	return d_min + d_mid - (tc_max - ttc) / (tc_max - tc_mid);
}

void Karamouzas::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("alpha", alpha);
	params.ReadFloat("beta", beta);
	params.ReadFloat("gamma", gamma);
	params.ReadFloat("delta", delta);
	params.ReadFloat("t_max", t_max);
}