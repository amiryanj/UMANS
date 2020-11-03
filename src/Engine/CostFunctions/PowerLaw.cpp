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

#include <CostFunctions/PowerLaw.h>
#include <core/agent.h>
#include <core/worldBase.h>

#define _EPSILON 0.00001f

Vector2D PowerLaw::ComputeAgentInteractionForce(const Agent* agent, const PhantomAgent& other) const
{
	// Custom implementation of the Power Law force, based on the paper's supplementary material:
	// https://journals.aps.org/prl/supplemental/10.1103/PhysRevLett.113.238701/prl_supplemental.pdf

	// This is based on a quadratic equation very similar to the one solved by ComputeTimeToCollision().
	// Some constants appear to be wrong, but we have kept this to match the paper as closely as possible.

	float R = other.realAgent->getRadius() + agent->getRadius();
	const Vector2D& x = agent->getPosition() - other.GetPosition();
	const Vector2D& v = agent->getVelocity() - other.GetVelocity();
	float a = v.dot(v);
	float b = -x.dot(v);
	float c = x.dot(x) - R * R;

	// ignore collisions that are already happening; our framework handles those via contact forces
	if (c <= 0)
		return Vector2D(0, 0);

	// compute time to collision; this derivation seems incorrect, but let's use it for the sake of reproducing the paper
	float d = b * b - a * c;
	if (d <= 0) // no solution
		return Vector2D(0, 0);
	float sqrtD = sqrt(d);
	float tau = (b - sqrtD) / a;

	// ignore collisions that lie in the past
	if (tau < 0.001f)
		return Vector2D(0, 0);

	// ignore collisions that lie too far away
	if (tau > tau0)
		return Vector2D(0, 0);

	float component1 = -k * exp(-tau / tau0) * (2 / tau + 1 / tau0) / (a * tau*tau);
	const Vector2D& component2 = v - (a * x + b * v) / sqrtD;

	return component1 * component2;
}

Vector2D PowerLaw::ComputeObstacleInteractionForce(const Agent* agent, const LineSegment2D& obstacle) const
{
	// TODO: implement obstacle force
	return Vector2D(0, 0);
}

void PowerLaw::parseParameters(const CostFunctionParameters & params)
{
	ForceBasedFunction::parseParameters(params);
	params.ReadFloat("k", k);
	params.ReadFloat("tau0", tau0);
}
