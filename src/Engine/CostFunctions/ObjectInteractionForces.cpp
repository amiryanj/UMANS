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

#include <CostFunctions/ObjectInteractionForces.h>
#include <core/agent.h>
#include <core/worldBase.h>

Vector2D ObjectInteractionForces::ComputeForce(Agent* agent, const WorldBase * world) const
{
	const Vector2D& Position = agent->getPosition();
	const float rangeSquared = range_ * range_;

	// loop over all neighbors; sum up the forces per neighbor
	Vector2D AgentForces(0, 0);
	const auto& neighbors = agent->getNeighbors();
	for (const PhantomAgent& other : neighbors.first)
	{
		if (other.GetDistanceSquared() <= rangeSquared)
			AgentForces += ComputeAgentInteractionForce(agent, other);
	}

	Vector2D ObstacleForces(0, 0);
	for (const LineSegment2D& obs : neighbors.second)
	{
		if (distanceToLineSquared(Position, obs.first, obs.second, true) <= rangeSquared)
			ObstacleForces += ComputeObstacleInteractionForce(agent, obs);
	}

	return AgentForces + ObstacleForces;
}
