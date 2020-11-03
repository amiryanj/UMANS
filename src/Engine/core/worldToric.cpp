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

#include <core/worldToric.h>

using namespace nanoflann;
using namespace std;

WorldToric::WorldToric(float width, float height) 
	: WorldBase(TORIC_WORLD), width_(width), height_(height)
{
}

void WorldToric::computeNeighbors_Displaced(const Vector2D& position, const Vector2D& displacement, float search_radius, const Agent* queryingAgent,
	NeighborList& result) const
{
	Vector2D minDisplacement(-displacement);
	
	if (agentKDTree != nullptr)
	{
		// compute neighboring agents
		vector<const Agent*> agents;
		computeNeighboringAgents_Flat(position + displacement, search_radius, queryingAgent, agents);

		// efficiently add them to the result
		size_t oldSize = result.first.size(), extraSize = agents.size();
		result.first.resize(oldSize + extraSize);
		for (size_t i = 0; i < extraSize; ++i)
			result.first[oldSize + i] = PhantomAgent(agents[i], position, minDisplacement);
	}

	// compute neighboring obstacles
	vector<LineSegment2D> obstacles;
	computeNeighboringObstacles_Flat(position + displacement, search_radius, obstacles);

	// efficiently add them to the result
	size_t oldSize = result.second.size(), extraSize = obstacles.size();
	result.second.resize(oldSize + extraSize);
	for (size_t i = 0; i < extraSize; ++i)
		result.second[oldSize + i] = LineSegment2D(obstacles[i].first - displacement, obstacles[i].second - displacement);
}

NeighborList WorldToric::ComputeNeighbors(const Vector2D& position, float search_radius, const Agent* queryingAgent) const
{
	NeighborList result;
	computeNeighbors_Displaced(position, Vector2D(0, 0), search_radius, queryingAgent, result);

	if (position.x - search_radius < -0.5*width_)
		computeNeighbors_Displaced(position, Vector2D(width_, 0), search_radius, queryingAgent, result);
	
	if (position.x + search_radius > 0.5*width_)
		computeNeighbors_Displaced(position, Vector2D(-width_, 0), search_radius, queryingAgent, result);
	
	if (position.y - search_radius < -0.5*height_)
		computeNeighbors_Displaced(position, Vector2D(0, height_), search_radius, queryingAgent, result);

	if (position.y + search_radius > 0.5*height_)
		computeNeighbors_Displaced(position, Vector2D(0, -height_), search_radius, queryingAgent, result);

	return result;
}

void WorldToric::DoStep_MoveAllAgents()
{
	const float halfWidth = 0.5f * width_;
	const float halfHeight = 0.5f * height_;

#pragma omp parallel for
	for (int i = 0; i < (int)agents_.size(); i++)
	{
		Agent* agent = agents_[i];
		
		// update the agent's velocity and position as usual
		agent->UpdateVelocityAndPosition(delta_time_);
		
		// if the agent has crossed the bounding rectangle, warp it to the other side
		float x = agent->getPosition().x;
		float y = agent->getPosition().y;
		if (x > halfWidth)
			x -= width_;
		else if (x < -halfWidth)
			x += width_;
		if (y > halfHeight)
			y -= height_;
		else if (y < -halfHeight)
			y += height_;

		agent->setPosition(Vector2D(x,y));
	}
}
