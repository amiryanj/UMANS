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

#include <CostFunctions/Paris.h>
#include <core/worldBase.h>
#include <core/agent.h>

using namespace std;

float Paris::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const Vector2D& direction = velocity.getnormalized();

	// Compute t1 and t2 for each neighbor, as defined in this paper:
	// the agent will avoid a neighbor by passing in front of it (before t1) or behind it (after t2).
	const auto& neighborAvoidanceTimes = ComputeAllNeighborAvoidanceRanges(direction, agent);

	float directionCost = GetDirectionCost(direction, agent, neighborAvoidanceTimes);
	float speedDeviationCost = GetSpeedDeviationCost(velocity.magnitude(), direction, agent, neighborAvoidanceTimes);
	float diffToPreferred = (velocity - agent->getPreferredVelocity()).magnitude();

	return 1000 * directionCost + speedDeviationCost + 0.001f * diffToPreferred;
}

std::pair<float, float> Paris::ComputeT1andT2(const Vector2D& origin, const Vector2D& velocity, const Vector2D& point, const float radius) const
{
	// t1 and t2 are the times at which the neighboring agent starts and stops touching 'point', a position on the neighbor's predicted trajectory.
	// To find them, we must solve the following equation for t:
	//    || origin + velocity*t - point || = radius
	// We can re-use the time-to-collision solution to find t1:
	float t1 = ComputeTimeToCollision(origin, velocity, radius, point, Vector2D(0, 0), 0);
	if (t1 > t_max)
		t1 = t_max;

	// Then t2 lies just after it:
	float t2 = t1 + radius / velocity.magnitude();
	if (t2 > t_max)
		t2 = 0;

	return { t1, t2 };

	/*// solve the following equation for t:
	//     || origin + velocity*t - point || = radius
	// =>  || dp + velocity*t || = radius (where dp = origin - point)
	// =>  (dp.x + velocity.x*t)^2 + (dp.y + velocity.y*t)^2 = radius^2
	// =>  dp.x^2 + 2*dp.x*velocity.x + velocity.x*t^2 + [same for y] = radius^2
	// =>  [dp.x*dp.x + dp.y*dp.y] + 2*[dp.x*velocity.x + dp.y*velocity.y] * t + [velocity.x*velocity.x + velocity.y*velocity.y] * t^2 = radius*radius
	// => dot(velocity, velocity) * t^2 + 2*dot(dp, velocity)*t + dot(dp, dp) - radius*radius = 0
	
	Vector2D dp(origin - point);
	float a = velocity.dot(velocity);
	float b = 2 * dp.dot(velocity);
	float c = dp.dot(dp) - radius * radius;

	float t1 = MaxFloat, t2 = MaxFloat;
	const int nrSolutions = SolveQuadraticEquation(a, b, c, t1, t2);

	float T1 = std::min(t1, t2);
	float T2 = std::max(t1, t2);

	// ignore solutions that lie in the past or too far in the future
	if (T1 < 0 || T1 > t_max)
	{
		T1 = t_max;
		T2 = 0;
	}

	return { T1, T2 };*/
}

Paris::NeighborAvoidanceRange Paris::ComputeNeighborAvoidanceRange(const Vector2D& direction, const Agent* agent, const PhantomAgent& neighbor) const
{
	const auto& agentPos = agent->getPosition();
	const auto& neighborPos = neighbor.GetPosition();
	const float radius1 = agent->getRadius(), radius2 = neighbor.realAgent->getRadius();

	// Find the point where the trajectories of 'agent' and 'neighbor' intersect
	Vector2D X;
	bool intersects = getLineIntersection(
		agentPos, agentPos + direction,
		neighborPos, neighborPos + neighbor.GetVelocity(),
		X);

	// If there is no intersection, two things could be going on:
	// a) the agents are not on collision course, and this direction is entirely safe;
	// b) the agents are on collision course even though X is not well-defined, and this direction may not be safe after all.
	if (!intersects)
	{
		// check if the agents are on collision course
		float ttc = ComputeTimeToCollision(agentPos, direction, radius1, neighborPos, neighbor.GetVelocity(), radius2);

		// if not (or if it's too far in the future), this direction is fine
		if (ttc > t_max)
			return NeighborAvoidanceRange(t_max, 0, 0, std::numeric_limits<float>::max());

		// otherwise, there will be a collision, and this direction is not fine
		else return NeighborAvoidanceRange(0, t_max, std::numeric_limits<float>::max(), 0);
	}
		

	// if the intersection is behind 'agent', ignore it as well
	const auto& t12_agent = ComputeT1andT2(agentPos, agent->getVelocity(), X, radius1+radius2); 
	if (t12_agent.first == t_max)
		return NeighborAvoidanceRange(t_max, 0, 0, std::numeric_limits<float>::max());
	
	// compute t1 and t2, the times at which 'neighbor' starts and stops touching the point X
	const auto& t12 = ComputeT1andT2(neighborPos, neighbor.GetVelocity(), X, radius1 + radius2);

	// compute s1 and s2, the speeds that 'agent' should use to reach X after exactly t1 and t2 seconds
	float distToX = (X - agentPos).magnitude();
	float s1 = distToX / t12.first;
	float s2 = distToX / t12.second;

	return NeighborAvoidanceRange(t12.first, t12.second, s1, s2);
}

Paris::NeighborAvoidanceRangeList Paris::ComputeAllNeighborAvoidanceRanges(const Vector2D& direction, const Agent* agent) const
{
	NeighborAvoidanceRangeList result;
	const Vector2D& Position = agent->getPosition();
	const auto& neighbors = agent->getNeighbors();
	const float rangeSquared = range_ * range_;

	// agents
	for (const auto& neighbor : neighbors.first)
	{
		if (neighbor.GetDistanceSquared() > rangeSquared)
			continue;

		result.push_back(ComputeNeighborAvoidanceRange(direction, agent, neighbor));
	}

	// TODO: static obstacles?
	// ...

	return result;
}

float Paris::CSpeed(const float preferredSpeed, const NeighborAvoidanceRange& avoidance) const
{
	// penalize if the preferred speed does not lie between s1 and s2
	return std::min(
		std::max(0.0f, 1.0f - avoidance.s1/preferredSpeed),
		std::max(0.0f, (avoidance.s2 - preferredSpeed)/preferredSpeed)
	);
}

float Paris::CPred(const NeighborAvoidanceRange& avoidance) const
{
	// decrease the weight if the potential collision is far away
	return 1 - avoidance.t1 / (t_max + w_b);
}

float Paris::CAngle(const Vector2D& direction, const Vector2D& preferredVelocity) const
{
	// compute a weight based on the angle with the preferred velocity
	return (float)((1 - cosAngle(direction, preferredVelocity)) / 2.0);
}

float Paris::GetDirectionCost(const Vector2D& direction, const Agent* agent, const NeighborAvoidanceRangeList& avoidanceTimes) const
{
	const Vector2D& preferredVelocity = agent->getPreferredVelocity();
	float preferredSpeed = preferredVelocity.magnitude();

	// compute C_speed * C_pred per neighbor, and keep track of the sum
	float sum_CSpeed_CPred = 0;
	for (const auto& neighbor : avoidanceTimes)
		sum_CSpeed_CPred += CSpeed(preferredSpeed, neighbor) * CPred(neighbor);

	// compute the average
	if (!avoidanceTimes.empty())
		sum_CSpeed_CPred /= avoidanceTimes.size();
	
	// return a weighted average of the speed component and the angle component
	return w_a * sum_CSpeed_CPred + (1 - w_a) * CAngle(direction, preferredVelocity);
}

float Paris::GetSpeedDeviationCost(const float speed, const Vector2D& direction, const Agent* agent, const NeighborAvoidanceRangeList& avoidanceRanges) const
{
	float preferredSpeed = agent->getPreferredVelocity().magnitude();

	// per neighbor, compute the max difference to the "avoidance speeds" s1 and s2
	
	float sumOfDeviations = 0;
	for (const auto& range : avoidanceRanges)
	{
		sumOfDeviations += std::max(
			0.0f, 
			std::max((range.s1 - speed) / preferredSpeed, 
			(speed - range.s2) / preferredSpeed)
		);
	}

	return sumOfDeviations;
}

void Paris::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("w_a", w_a);
	params.ReadFloat("w_b", w_b);
	params.ReadFloat("t_max", t_max);
}