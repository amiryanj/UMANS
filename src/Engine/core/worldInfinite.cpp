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

#include <core/worldInfinite.h>

using namespace nanoflann;
using namespace std;

WorldInfinite::WorldInfinite() : WorldBase(INFINITE_WORLD)
{
}

NeighborList WorldInfinite::ComputeNeighbors(const Vector2D& position, float search_radius, const Agent* queryingAgent) const
{
	vector<PhantomAgent> phantoms; 
	if (agentKDTree != nullptr)
	{
		// compute neighboring agents
		vector<const Agent*> agents;
		computeNeighboringAgents_Flat(position, search_radius, queryingAgent, agents);
		
		// efficiently add them to the result
		Vector2D offset(0, 0);
		phantoms.resize(agents.size());
		for (size_t i = 0; i < agents.size(); ++i)
			phantoms[i] = PhantomAgent(agents[i], position, offset);
	}

	// compute neighboring obstacles
	vector<LineSegment2D> obstacles;
	computeNeighboringObstacles_Flat(position, search_radius, obstacles);

	return { phantoms, obstacles };
}

