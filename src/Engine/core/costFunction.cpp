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

#include <core/costFunction.h>

#include <core/worldBase.h>
#include <tools/Matrix.h>

CostFunction::CostFunction() {}
CostFunction::~CostFunction() {}

Vector2D CostFunction::RotateGradientToEuclideanCoordinates(const float GradTh, const float GradS, const Vector2D& direction, const float speed) const
{
	Matrix R(Vector2D(direction.y, -direction.x), direction);
	return R * (Vector2D(sin(GradTh), 1 - cos(GradTh))*speed + Vector2D(0, GradS));
}

Vector2D CostFunction::GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase* world) const
{
	// Default implementation of the gradient: a numerical approximation

	float delta = 0.1f;
	Vector2D deltaX(delta, 0), deltaY(0, delta);

	// approximate gradient in x direction
	float costLeft = GetCost(velocity - deltaX, agent, world);
	float costRight = GetCost(velocity + deltaX, agent, world);
	float gradientX = (costRight - costLeft) / (2 * delta);

	// approximate gradient in y direction
	float costBottom = GetCost(velocity - deltaY, agent, world);
	float costTop = GetCost(velocity + deltaY, agent, world);
	float gradientY = (costTop - costBottom) / (2 * delta);

	return Vector2D(gradientX, gradientY);
}

Vector2D CostFunction::GetGlobalMinimum(Agent* agent, const WorldBase* world) const
{
	// By default, we approximate the global optimum via sampling.
	return ApproximateGlobalMinimumBySampling(
		agent, world, 
		SamplingParameters::ApproximateGlobalOptimization(), 
		{ { this, 1.0f } }
	);
}

Vector2D CostFunction::ApproximateGlobalMinimumBySampling(Agent* agent, const WorldBase* world,
	const SamplingParameters& params, const CostFunctionList& costFunctions)
{
	// --- Compute the range in which samples will be taken.

	// compute the base of the cone, or center of the circle
	Vector2D base(0,0);
	if (params.base == SamplingParameters::Base::ZERO) base = Vector2D(0, 0);
	else if (params.base == SamplingParameters::Base::CURRENT_VELOCITY) base = agent->getVelocity();

	// compute the radius of the cone/circle
	float radius;
	if (params.radius == SamplingParameters::Radius::PREFERRED_SPEED) radius = agent->getPreferredSpeed();
	else if (params.radius == SamplingParameters::Radius::MAXIMUM_SPEED) radius = agent->getMaximumSpeed();
	else if (params.radius == SamplingParameters::Radius::MAXIMUM_ACCELERATION) 
		radius = std::min(2.0f*agent->getMaximumSpeed(), agent->getMaximumAcceleration() * world->GetDeltaTime());

	// compute the base direction (a unit vector)
	Vector2D baseDirection(1, 0);
	if (params.baseDirection == SamplingParameters::BaseDirection::UNIT) baseDirection = Vector2D(1, 0);
	else if (params.baseDirection == SamplingParameters::BaseDirection::CURRENT_VELOCITY) baseDirection = agent->getVelocity().getnormalized();
	else if (params.baseDirection == SamplingParameters::BaseDirection::PREFERRED_VELOCITY) baseDirection = agent->getPreferredVelocity().getnormalized();

	// compute the maximum angle to the base direction, in radians
	float maxAngle = (float)(params.angle / 360.0 * PI); // params.angle stores the full range (in deg); we want half of it (in rad)

	// --- Option 1: Random sampling

	Vector2D bestVelocity(0, 0);
	float bestCost = MaxFloat;

	if (params.type == SamplingParameters::Type::RANDOM)
	{
		for (int i = 0; i < params.randomSamples; ++i)
		{
			// create a random velocity in the cone/circle
			float randomAngle = agent->ComputeRandomNumber(-maxAngle, maxAngle);
			float randomLength = agent->ComputeRandomNumber(0, radius);
			const Vector2D& velocity = base + rotateCounterClockwise(baseDirection, randomAngle) * randomLength;

			// compute the cost for this velocity
			float totalCost = 0;
			for (auto& costFunction : costFunctions)
				totalCost += costFunction.second * costFunction.first->GetCost(velocity, agent, world);

			// check if this cost is better than the minimum so far
			if (totalCost < bestCost)
			{
				bestVelocity = velocity;
				bestCost = totalCost;
			}
		}
	}

	// --- Option 2: Regular sampling

	else if (params.type == SamplingParameters::Type::REGULAR)
	{
		// compute the difference in angle and length per iteration
		const float startAngle = -maxAngle;
		const float endAngle = maxAngle;
		const float deltaAngle = (endAngle - startAngle) / (params.angle == 360 ? params.angleSamples : (params.angleSamples - 1));
		const float deltaLength = radius / (params.includeBaseAsSample ? (params.speedSamples - 1) : params.speedSamples);

		// speed samples
		for (int s = 1; s <= params.speedSamples; ++s)
		{
			const float candidateLength = deltaLength * (params.includeBaseAsSample ? s-1 : s);
			float candidateAngle = startAngle;

			// angle samples
			for (int a = 0; a < params.angleSamples; ++a, candidateAngle += deltaAngle)
			{
				// construct the candidate velocity
				const Vector2D& velocity = base + rotateCounterClockwise(baseDirection, candidateAngle) * candidateLength;

				// compute the cost for this velocity
				float totalCost = 0;
				for (auto& costFunction : costFunctions)
					totalCost += costFunction.second * costFunction.first->GetCost(velocity, agent, world);

				// check if this cost is better than the minimum so far
				if (totalCost < bestCost)
				{
					bestVelocity = velocity;
					bestCost = totalCost;
				}

				// if we are currently checking the base velocity, we don't have to sample any more angles
				if (params.includeBaseAsSample && s == 1)
					break;
			}
		}
	}

	// --- Return the velocity with the lowest cost

	return bestVelocity;
}

void CostFunction::parseParameters(const CostFunctionParameters & params)
{
	params.ReadFloat("range", range_);
}

// Solves a quadratic equation of the form ax^2 + bx + c = 0.
// Returns the number of solutions, and possibly fills in answer1 and answer2 depending on the number of solutions.
int CostFunction::SolveQuadraticEquation(float a, float b, float c, float& answer1, float& answer2) const
{
	const float D = b * b - 4 * a*c;
	if (D < 0)
		return 0;
	if (D == 0)
	{
		answer1 = -b / 2 * a;
		return 1;
	}
	else
	{
		const float minusBdiv2A = -b / (2 * a);
		const float sqrtDdiv2A = sqrt(D) / (2 * a);
		answer1 = minusBdiv2A + sqrtDdiv2A;
		answer2 = minusBdiv2A - sqrtDdiv2A;
		return 2;
	}
}

float CostFunction::ComputeTimeToCollision(
	const Vector2D& position1, const Vector2D& velocity1, const float radius1,
	const Vector2D& position2, const Vector2D& velocity2, const float radius2) const
{
	const Vector2D PDiff(position1 - position2);
	const float Radii = radius1 + radius2;
	const float RadiiSq = Radii * Radii;

	// is there already a collision now?
	if (PDiff.sqrMagnitude() <= RadiiSq)
		return 0;
	
	const Vector2D VDiff(velocity1 - velocity2);

	// To find out at what time the two moving disks first intersect, we must solve the following equation:
	//      dist(p1 + v1*t, p2 + v2*t) = r1 + r2
	// =>   || PDiff + VDiff*t || = Radii
	// =>   || PDiff + VDiff*t ||^2 = Radii^2
	// =>   V.x^2*t^2 + 2*P.x*V.x*t + P.x^2 + (same for .y) = R^2
	// =>   (V.x*V.x + V.y*V.y) * t^2 + 2*(P.x*V.x + P.y*V.y) * t + (P.x*P.x + P.y*P.y) = R^2
	// =>   (V.V)*t^2 + 2*(P.)*t + (P.P) = R^2
	const float a = VDiff.dot(VDiff);
	const float b = 2 * PDiff.dot(VDiff);
	const float c = PDiff.dot(PDiff) - RadiiSq;

	float t1 = MaxFloat, t2 = MaxFloat;
	const int nrSolutions = SolveQuadraticEquation(a, b, c, t1, t2);

	// ignore solutions that lie in the past
	if (t1 < 0) t1 = MaxFloat;
	if (t2 < 0) t2 = MaxFloat;

	// choose the solution that occurs first; could be MaxDouble if there are no solutions
	return std::min(t1, t2);
}

float CostFunction::ComputeTimeToFirstCollision(const Vector2D& position, const Vector2D& velocity, const float radius,
	const NeighborList& neighbors, const float maximumDistance, bool ignoreCurrentCollisions) const
{
	float minTTC = MaxFloat;
	const float maxDistSquared = maximumDistance * maximumDistance;

	// check neighboring agents
	for (const auto& neighborAgent : neighbors.first)
	{
		if (neighborAgent.GetDistanceSquared() > maxDistSquared)
			continue;

		float ttc = ComputeTimeToCollision(position, velocity, radius, neighborAgent.GetPosition(), neighborAgent.GetVelocity(), neighborAgent.realAgent->getRadius());
		
		// ignore current collisions?
		if (ignoreCurrentCollisions && ttc == 0)
			continue;

		if (ttc < minTTC)
			minTTC = ttc;
	}

	// check neighboring obstacles
	for (const auto& neighboringObstacle : neighbors.second)
	{
		if (distanceToLineSquared(position, neighboringObstacle.first, neighboringObstacle.second, true) > maxDistSquared)
			continue;

		float ttc = ComputeTimeToCollision_LineSegment(position, velocity, radius, neighboringObstacle);

		// ignore current collisions?
		if (ignoreCurrentCollisions && ttc == 0)
			continue;

		if (ttc < minTTC)
			minTTC = ttc;
	}

	return minTTC;
}

float CostFunction::ComputeTimeToCollision_LineSegment(const Vector2D& position, const Vector2D& velocity, const float radius, const LineSegment2D& lineSegment) const
{
	// compute TTC for the first endpoint
	float timeToEndpoint1 = ComputeTimeToCollision(position, velocity, radius, lineSegment.first, Vector2D(0,0), 0);
	if (timeToEndpoint1 == 0)
		return 0;

	// if the line segment is actually a point, return the only TTC we've computed
	if (lineSegment.first == lineSegment.second)
		return timeToEndpoint1;

	// compute TTC for the second endpoint
	float timeToEndpoint2 = ComputeTimeToCollision(position, velocity, radius, lineSegment.second, Vector2D(0, 0), 0);
	if (timeToEndpoint2 == 0)
		return 0;

	// compute TTC for the interior of the segment
	float timeToInterior = ComputeTimeToCollision_LineSegmentInterior(position, velocity, radius, lineSegment);

	// return the smallest of those three
	return std::min(std::min(timeToEndpoint1, timeToEndpoint2), timeToInterior);
}

float CostFunction::ComputeTimeToCollision_LineSegmentInterior(const Vector2D& position, const Vector2D& velocity, const float radius, const LineSegment2D& lineSegment) const
{
	// Check if the disk is already colliding with the line segment right now
	if (distanceToLineSquared(position, lineSegment.first, lineSegment.second, true) <= radius*radius)
		return 0;

	// Compute the intersection of the disk's future trajectory and the line segment, ignoring the disk's radius for now (we will correct this later)
	Vector2D intersection;
	bool intersects = getLineIntersection(position, position + velocity, lineSegment.first, lineSegment.second, intersection);
	// - check if there is no intersection at all
	if (!intersects)
		return MaxFloat;
	// - check if the intersection point lies behind the agent
	if (velocity.dot(intersection - position) < 0)
		return MaxFloat;
	// - check if the intersection point actually lies on the segment
	bool pointIsOnSegment;
	nearestPointOnLine(intersection, lineSegment.first, lineSegment.second, false, &pointIsOnSegment);
	if (!pointIsOnSegment)
		return MaxFloat;

	// Compute the time after which this intersection will occur
	float speed = velocity.magnitude();
	float timeToCollision = (intersection - position).magnitude() / speed;

	// We now know when a *point* agent would hit the line segment, but a *disk* agent will hit it sooner.
	// Apply a correction for this:
	float lineAngle = angle(velocity, lineSegment.second - lineSegment.first);
	float distanceToSubtract = radius / sinf(lineAngle);
	float timeToSubtract = distanceToSubtract / speed;

	return std::max(0.0f, timeToCollision - timeToSubtract);
}

/*std::pair<float, float> CostFunction::ComputeTimeAndDistanceToClosestApproach(
	const Vector2D& position1, const Vector2D& velocity1, const float radius1,
	const Vector2D& position2, const Vector2D& velocity2, const float radius2) const
{
	// dp = difference in position
	Vector2D dp(position2 - position1);

	// dv = different in velocity
	Vector2D dv(velocity2 - velocity1);

	// ttca = time to closest approach. Source: Dutra et al, "Gradient-based steering for vision-based crowd simulation algorithms", 2016.
	float ttca = (dv.sqrMagnitude() == 0 ? 0 : -dp.dot(dv) / dv.sqrMagnitude());

	// The "classical" definition of ttca does not take agent radii into account,
	// and in case of a collision, it computes the time to the biggest overlap.
	// This affects cost functions incorrectly. We use the following variant:
	// if there is a future collision (i.e. ttc is defined), then ttca = tca. Otherwise, ttca = the classical version.

	float ttc = ComputeTimeToCollision(position1, velocity1, radius1, position2, velocity2, radius2);
	ttca = std::min(ttca, ttc);

	float dca = (dp + dv * ttca).magnitude() - radius1 - radius2;
	if (dca < 0) dca = 0;

	return { ttca, dca };
}*/

std::pair<float, float> CostFunction::ComputeTimeAndDistanceToClosestApproach(
	const Vector2D& position1, const Vector2D& velocity1, const float radius1,
	const Vector2D& position2, const Vector2D& velocity2, const float radius2) const
{
	const Vector2D dp_center(position2 - position1);
	const Vector2D dv_center(velocity2 - velocity1);
	
	// The standard TTCA definition works with these dp and dv immediately.
	// We make some corrections to account for the radii of agents:
	float dpMag = dp_center.magnitude();
	const Vector2D& dp = dp_center / dpMag * (dpMag - radius1 - radius2);
	Vector2D dpNormal(-dp_center.y, dp_center.x);
	const Vector2D dv = dv_center + dpNormal * dpNormal.dot(dv_center);

	// Source: Dutra et al, "Gradient-based steering for vision-based crowd simulation algorithms", 2016.
	float ttca = (dv.sqrMagnitude() == 0 ? 0 : -dp.dot(dv) / dv.sqrMagnitude());
	// Prevent negative ttca?
	// This means that if the agents are moving away from each other, it does not matter how strongly they do this
	if (ttca < 0) ttca = 0;

	float dca = (dp + dv * ttca).magnitude(); // subtracting the radii is not needed anymore, dp and dv already incorporate this
	// Prevent negative dca? 
	// Combined with a non-negative ttca, this can only happen if agents are already colliding now.
	if (dca < 0) dca = 0;

	return { ttca, dca };
}