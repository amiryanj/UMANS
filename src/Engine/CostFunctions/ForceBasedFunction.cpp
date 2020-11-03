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

#include <CostFunctions/ForceBasedFunction.h>
#include <core/agent.h>
#include <core/worldBase.h>

float ForceBasedFunction::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const Vector2D& targetV = ComputeTargetVelocity(agent, world);
	return 0.5f * (velocity - targetV).sqrMagnitude() / world->GetDeltaTime();
}

Vector2D ForceBasedFunction::GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	const Vector2D& targetV = ComputeTargetVelocity(agent, world);
	return (velocity - targetV) / world->GetDeltaTime();
}

Vector2D ForceBasedFunction::GetGlobalMinimum(Agent* agent, const WorldBase* world) const
{
	return ComputeTargetVelocity(agent, world);
}

void ForceBasedFunction::parseParameters(const CostFunctionParameters & params)
{
	CostFunction::parseParameters(params);
	params.ReadFloat("agentMass", agentMass);
}

Vector2D ForceBasedFunction::ComputeTargetVelocity(Agent* agent, const WorldBase* world) const
{
	return agent->getVelocity() + ComputeForce(agent, world) / agentMass * world->GetDeltaTime();
}