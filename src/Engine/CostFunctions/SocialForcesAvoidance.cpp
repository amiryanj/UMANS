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

#include <CostFunctions/SocialForcesAvoidance.h>
#include <core/agent.h>
#include <core/worldBase.h>

Vector2D SocialForcesAvoidance::ComputeAgentInteractionForce(const Agent* agent, const PhantomAgent& other) const
{
	// This is an implementation of equations from the paper by Helbing and Molnar:
	//
	// R = agent.position - other.position,
	// F = -nabla_R V(b(R)), {Eq. 3}
	// where V is a scalar potential function: 
	// V(b(R)) = V0 * e^(-b(R) / sigma), {Eq. 13}
	// where b is the axis of an ellipse: 
	// b(R) = 0.5 * sqrt[(||R|| + ||R - vb*dt*Eb||)^2 - (vb*dt)^2]. {Eq. 4}
	//
	// Unfortunately, the math in the paper seems ambiguous. 
	// We assume that nabla_R V(b(R)) means "directional derivative of V(b(R)) with respect to vector R",
	// so Eq. 3 can be rewritten to:
	// F = -R * d/dR V(b(R))
	//   = -R * d/dR [ V0 * e^(-b(R) / sigma) ]
	//   = -R * V0 * -1 / sigma * e^(-b(R) / sigma) * d/dR b(R)
	//   =  R * V0/sigma * e^(-b(R) / sigma) * d/dR b(R)
	//
	// This means that the force always points exactly in the opposite direction of R.
	// The complicated part is finding its *magnitude*.

	const Vector2D& R = agent->getPosition() - other.GetPosition();
	float magR = R.magnitude();

	// if the agents are already colliding right now, don't apply any force; we'll do this via contact forces instead
	if (magR <= agent->getRadius() + other.realAgent->getRadius())
		return Vector2D(0, 0);

	// preferred velocity of the other agent
	float vb = other.GetVelocity().magnitude();
	const Vector2D& Eb = other.realAgent->getPreferredVelocity().getnormalized();

	// B = semi-minor axis of an ellipse, describing the repulsive potential,
	// defined in Eq. 4 of the paper as: b(R) = 0.5 * sqrt[(||R|| + ||R - vb*dt*Eb||)^2 - (vb*dt)^2],
	// so (2b)^2 = (||R|| + ||R - vb*dt*Eb||)^2 - (vb*dt)^2
	const Vector2D& V = vb * dt*Eb;
	float magV = V.magnitude();
	const Vector2D& R2 = R - V;
	float magR2 = R2.magnitude();
	float bSquared2 = pow(magR + magR2, 2) - pow(vb*dt, 2);
	if (bSquared2 <= 0)
		return Vector2D(0, 0);

	float b = 0.5f*sqrtf(bSquared2);
	if (b < 0.001f)
		return Vector2D(0, 0);

	// The closed form of d/dR b(R) is never reported in literature, so perhaps it is considered trivial.
	// Because we do not agree with that, here is our expansion: 
	//
	// d/dR b(R) = d/dR [ 1/2 * sqrt[(||R|| + ||R - V||)^2 - (vb*dt)^2] ]
	//           = d/dR [ 1/2 * sqrt(bSquared2(R)) ]                 { def. of bSquared2 as computed above }
	//           = 1/(4*sqrt(bSquared2(R))) * d/dR bSquared2(R)      { chain rule }
	//           = 1/(4*2b(R)) * d/dR bSquared2(R)                   { because sqrt(bSquared2(R)) = 2*b(R) }
	//           = 1/(8b(R))   * d/dR bSquared2(R)
	//
	// where
	// d/dR bSquared2(R) = 2 * (||R|| + ||R - V||) * [ d/dR ||R|| + d/dR ||R - V|| ]    { chain rule }
	//                   = 2 * (||R|| + ||R - V||) * [  1         + d/dR ||R - V|| ]
	//
	// where
	// d/dR ||R - V|| = 1/(2*||R - V||) * d/dR (||R - V||)                         { chain rule }
	// d/dR ||R - V|| = 1/(2*||R - V||) * d/dR (||R||^2 + ||V||^2 - 2.R.V)         { expansion of ||R - V|| }
	//                = 1/(2*||R - V||) * [d/dR ||R||^2 - 2*d/dR R.V]
	//                = 1/(2*||R - V||) * [2||R||       - 2||V||*cos(angle(R,V)) ]

	// Compute these derivatives bottom-up:
	const float d_magR2 = 1 / (2 * magR2) * (2 * magR - 2 * magV *cosAngle(R, V));

	const float d_bSquared2 = 2 * (magR + magR2) * (1 + d_magR2);

	const float d_b = (1 / (8 * b)) * d_bSquared2;

	const float d_V = -V0 / sigma * exp(-b / sigma) * d_b;

	// finally, compute the force
	const Vector2D& Force = -R * d_V;

	// scale the force down if the other agent is outside our view
	const float viewingAngleHalf = (float)(100.0 * PI / 180.0);
	const float ScaleOutsideView = 0.5f;
	float Scale = (angle(agent->getVelocity(), -R) < viewingAngleHalf ? 1 : ScaleOutsideView);
	return Scale * Force;
}

Vector2D SocialForcesAvoidance::ComputeObstacleInteractionForce(const Agent* agent, const LineSegment2D& obstacle) const
{
	// find the nearest point on the obstacle
	const Vector2D& nearest = nearestPointOnLine(agent->getPosition(), obstacle.first, obstacle.second, true);
	const Vector2D& diff = agent->getPosition() - nearest;

	// if the distance is extremely small, ignore it
	float dist = diff.magnitude();
	if (dist < 0.001f)
		return Vector2D(0, 0);

	// Note: just like in ComputeAgentInteractionForce(), the math by Helbing and Molnar seems ambiguous.
	// Again, we interpret nabla_R U(R) as "directional derivative of U(R) with respect to vector R".
	//
	// F = -nabla U(R), { Eq. 5 in the paper }
	//   = -R * d/dR U(R)
	// where U(R) is a scalar potential function:
	// U(R) = U0 * e^(-||R|| / sigma). { Eq. 13 }
	// 
	// d/dR U(R) = -U0/sigma * e^(-||R|| / sigma) * d/dR ||R||
	//           = -U0/sigma * e^(-||R|| / sigma).
	//
	// (Watch out: the 'sigma' here is actually called 'R' in the paper and in the code, 
	// but our explanation uses naming that matches the one from ComputeAgentInteractionForce().)
	//
	float d_U = -U0 / R * exp(-dist / R);

	return -diff * d_U;
}

void SocialForcesAvoidance::parseParameters(const CostFunctionParameters & params)
{
	ForceBasedFunction::parseParameters(params);
	params.ReadFloat("dt", dt);
	params.ReadFloat("V0", V0);
	params.ReadFloat("sigma", sigma);
	params.ReadFloat("U0", V0);
	params.ReadFloat("R", R);
}
