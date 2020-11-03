/* UMANS: Unified Microscopic Agent Navigation Simulator
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettr�
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

#include <CostFunctions/TtcaDca.h>
#include <core/worldBase.h>
#include <core/agent.h>
#include <tools/Matrix.h>

using namespace std;

const float eps = 0.001f;

/*
 * The cost function for a single ttca-dca pair
 */
float TtcaDca::costForTtcaDca(float ttca, float dca) const
{
    return (float)exp(-0.5*( pow(ttca/sigTtca_,2) + pow(dca/sigDca_,2) ) );
}

float TtcaDca::getMovementCost(const Vector2D& velocity, const Agent* agent) const
{
	float alpha = angle(velocity, agent->getPreferredVelocity());
	float ds = (velocity.magnitude() - agent->getPreferredSpeed()); 
	
	float C_angle = (float)exp(-0.5 * pow(alpha / sigAngle_goal_, 2.0));
	float C_speed = (float)exp(-0.5 * pow(ds / sigSpeed_goal_, 2.0));

	return 1 - (C_angle + C_speed) / 2.0f;
}

std::pair<float, float> TtcaDca::getMovementCostGradient(const Vector2D& velocity, const Agent* agent) const
{
	float alpha = counterClockwiseAngle(velocity, agent->getPreferredVelocity());
	float ds = (velocity.magnitude() - agent->getPreferredSpeed()); 

	// Gradients of the movement cost C_m (Eq. 14 and 15 in Dutra et al.)
	double DC_angle = alpha / (2*sigAngle_goal_*sigAngle_goal_) * exp(-0.5 * pow(alpha / sigAngle_goal_, 2.0));
	double DC_speed = ds / (2*sigSpeed_goal_*sigSpeed_goal_) * exp(-0.5 * pow(ds / sigSpeed_goal_, 2.0));

	return { (float)DC_angle, (float)DC_speed };
}

float TtcaDca::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const float Radius = agent->getRadius();
	const Vector2D& Position = agent->getPosition();
	const float rangeSquared = range_ * range_;

	// --- movement towards the goal

	float MovementCost = getMovementCost(velocity, agent);

	// --- collision avoidance

	float ObstacleCost = 0; 
	float ObstacleCostScale = 0;

	const auto& neighbors = agent->getNeighbors();

	// for each agent of the neighbourhood
	for (const auto& neighbor : neighbors.first)
	{
		const auto& neighborPos = neighbor.GetPosition();
		if (neighbor.GetDistanceSquared() >= rangeSquared)
			continue;

		// Compute relative velocity and relative position
		const Vector2D& relPos = neighborPos - Position;
		const Vector2D& relVelocity = neighbor.GetVelocity() - velocity;

		// ignore neighbors that are behind the agent; the original Dutra method uses rendering, and agents always face forward
		if (angle(relPos, agent->getVelocity()) > viewingAngleHalf_)
			continue;

		// there is adaptation only if relative velocity is not zero
		// --> disabled because it makes the cost function non-smooth
		//if (relVelocity.sqrMagnitude() <= eps)
		//	continue;

		// computing ttc and dca
		const auto& ttca_dca = ComputeTimeAndDistanceToClosestApproach(
			Position, velocity, Radius,
			neighborPos, neighbor.GetVelocity(), neighbor.realAgent->getRadius());

		// ignore TTCAs in the past
		// --> disabled because it makes the cost function non-smooth
		//if (ttca_dca.first < 0) 
		//	continue;

		// The original method does this per pixel; we do it per obstacle.
		// To simulate the "number of pixels" for this obstacle, scale by the distance

		float distance = relPos.magnitude() - Radius - neighbor.realAgent->getRadius();
		float scale = 1 / (distance*distance); //simulate num of pixels of an agent in screen
		ObstacleCostScale += scale;

		float cost = costForTtcaDca(ttca_dca.first, ttca_dca.second) * scale;
		ObstacleCost += cost;
	}

	// TODO: check neighboring obstacles
	// ...
	
	if (ObstacleCostScale > 0)
		ObstacleCost /= ObstacleCostScale;

	// --- total
	
	return ObstacleCost + MovementCost;
}

Vector2D TtcaDca::GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const float Radius = agent->getRadius();
	const Vector2D& Position = agent->getPosition();
	const float rangeSquared = range_ * range_;

	// --- collision avoidance

	float GradTh = 0;
	float GradS = 0;
	float totalScale = 0;

	const auto& neighbors = agent->getNeighbors();

	// for each agent of the neighbourhood
	for (const auto& neighbor : neighbors.first)
	{
		if (neighbor.GetDistanceSquared() > rangeSquared)
			continue;

		// ------------------
		// --- The following code is very similar to CostFunction::ComputeTimeAndDistanceToClosestApproach,
		//     but we need certain components of it later in the calculation.

		const Vector2D dp_center(neighbor.GetPosition() - Position);
		const Vector2D dv_center(neighbor.GetVelocity() - velocity);

		// ignore neighbors that are behind the agent; the original Dutra method uses rendering, and agents always face forward
		if (angle(dp_center, agent->getVelocity()) > viewingAngleHalf_)
			continue;

		// The standard TTCA definition works with these dp and dv immediately.
		// We make some corrections to account for the radii of agents:
		float dpMag = dp_center.magnitude();
		const Vector2D& dp = dp_center / dpMag * (dpMag - Radius - neighbor.realAgent->getRadius());
		Vector2D dpNormal(-dp_center.y, dp_center.x);
		const Vector2D dv = dv_center + dpNormal * dpNormal.dot(dv_center);
		float dvSqrMagnitude = dv.sqrMagnitude();

		// Source: Dutra et al, "Gradient-based steering for vision-based crowd simulation algorithms", 2016.
		float ttca = (dvSqrMagnitude == 0 ? 0 : -dp.dot(dv) / dvSqrMagnitude);
		// Prevent negative ttca?
		// This means that if the agents are moving away from each other, it does not matter how strongly they do this
		if (ttca < 0) ttca = 0;

		const Vector2D vdca(dp + dv * ttca);
		float dca = vdca.magnitude(); // subtracting the radii is not needed anymore, dp and dv already incorporate this
		// Prevent negative dca? 
		// Combined with a non-negative ttca, this can only happen if agents are already colliding now.
		if (dca < 0) dca = 0;

		// --- End copy from CostFunction::ComputeTimeAndDistanceToClosestApproach
		// ------------------

		// If the relative velocity is almost zero, then the gradient is very unreliable; ignore it. 
		// This is dangerous in GetCost(), but acceptable for GetGradient().
		if (dvSqrMagnitude <= eps)
			continue;

		const Vector2D VelRot(dv.y, -dv.x);
		const float VelMagnitude = sqrt(dvSqrMagnitude);
		const Vector2D& VelNorm = dv / VelMagnitude;

		// Partial derivatives of TTCA (Eq. 29 and 30 of Dutra et al.)
		float gradTtcaAngle = 0, gradTtcaSpeed = 0;
		if (abs(ttca) > eps)
		{
			const Vector2D& rel = (dp + 2 * ttca*dv);
			gradTtcaAngle = -rel.dot(VelRot) / dvSqrMagnitude;
			gradTtcaSpeed = rel.dot(VelNorm) / dvSqrMagnitude;
		}
		// otherwise TTCA is too small, so gradient cannot be properly determined

		// Partial derivatives of DCA (Eq. 36 and 37 of Dutra et al.)
		float gradDcaAngle = 0, gradDcaSpeed = 0;
		if (abs(dca) > eps)
		{
			gradDcaAngle = vdca.dot(gradTtcaAngle*dv + ttca * VelRot) / dca;
			gradDcaSpeed = vdca.dot(gradTtcaSpeed*dv - ttca * VelNorm) / dca;
		}
		// otherwise DCA is too small, so gradient cannot be properly determined

		// Overall gradients of the obstacle cost function (Eq. 16 and 17 in Dutra et al.)
		float cost = costForTtcaDca(ttca, dca);
		float ttcaFrac = ttca / (sigTtca_*sigTtca_);
		float dcaFrac = dca / (sigDca_*sigDca_);
		float gradCSpeed = -cost * (gradTtcaSpeed * ttcaFrac + gradDcaSpeed * dcaFrac);
		float gradCAngle = -cost * (gradTtcaAngle * ttcaFrac + gradDcaAngle * dcaFrac);

		// The original method does this per pixel; we do it per obstacle.
		// To simulate the "number of pixels" for this obstacle, scale by the distance
		float scale = 1 / dp.sqrMagnitude();
		totalScale += scale;

		GradTh += gradCAngle * scale;
		GradS += gradCSpeed * scale;
	}

	// TODO: check neighboring obstacles
	// ...

	// The original method averages over all pixels. 
	// To simulate this, we should divide by the total scale we've applied per obstacle.
	if (totalScale > 0)
	{
		GradTh /= totalScale;
		GradS /= totalScale;
	}

	// --- add the "goal reaching" component

	const auto& GradThS_movement = getMovementCostGradient(velocity, agent);
	GradTh += GradThS_movement.first;
	GradS += GradThS_movement.second;
	
	// --- convert to a gradient in Euclidean velocity space

	Vector2D Gradient;

	if (velocity.sqrMagnitude() < 0.01)
	{
		// If the velocity is (almost) zero, then the transformation will return (nearly) (0,0) and the agent will never start moving.
		Gradient = RotateGradientToEuclideanCoordinates(GradTh, GradS,
			agent->getPreferredVelocity().getnormalized(), agent->getPreferredVelocity().magnitude());
	}
	else
	{
		Gradient = RotateGradientToEuclideanCoordinates(GradTh, GradS, velocity.getnormalized(), velocity.magnitude());
	}

	return Gradient;
}

void TtcaDca::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("sigmaAngle_goal", sigAngle_goal_);
	params.ReadFloat("sigmaSpeed_goal", sigSpeed_goal_);
	params.ReadFloat("sigmaTtca", sigTtca_);
	params.ReadFloat("sigmaDca", sigDca_);
}

