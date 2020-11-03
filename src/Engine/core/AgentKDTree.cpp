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

#include <core/AgentKDTree.h>

AgentKDTree::AgentKDTree(const std::vector<Agent*>& agents) : pointCloud(agents)
{
	kdTree = new NanoflannKDTree(2, pointCloud);
	kdTree->buildIndex();
}

AgentKDTree::~AgentKDTree()
{
	delete kdTree;
}

std::vector<size_t> AgentKDTree::FindAllAgentsInRange(const Vector2D& position, const float radius, const Agent* agentToIgnore) const
{
	// do a radius search in the kd-tree
	double q[2] = { position.x, position.y };
	std::vector<std::pair<size_t, double>> result_indicesAndDistances;
	nanoflann::SearchParams params; params.sorted = true;
	// note: nanoflann uses squared distances, so we search with radius*radius
	auto nrResults = kdTree->radiusSearch(q, radius*radius, result_indicesAndDistances, params);

	// convert the result to a list of agent IDs, possibly ignoring a certain agent
	std::vector<size_t> result(nrResults); size_t index = 0;
	for (size_t i = 0; i < nrResults; ++i)
	{
		const size_t pointCloudIndex = result_indicesAndDistances[i].first;
		const size_t agentID = pointCloud.agentPositions[pointCloudIndex].first;
		if (agentToIgnore != nullptr && agentID == agentToIgnore->getID())
			continue;
		result[index] = agentID;
		++index;
	}
	if (index < nrResults) 
		result.pop_back();

	return result;
}

std::vector<size_t> AgentKDTree::FindKNearestAgents(const Vector2D& position, const size_t k, const Agent* agentToIgnore) const
{
	std::vector<size_t> result;
	if (k == 0)
		return result;
		
	// when we want to ignore a certain agent, actually look for 1 agent more; we'll filter out the desired agent later
	size_t kSearch = k;
	if (agentToIgnore != nullptr)
		++kSearch;

	// do a knn search in the kd-tree
	double q[2] = { position.x, position.y };
	std::vector<size_t> result_indices(kSearch);
	std::vector<double> result_distances(kSearch);
	std::vector<std::pair<size_t, double>> result_indicesAndDistances;
	auto nrResults = kdTree->knnSearch(q, kSearch, &result_indices[0], &result_distances[0]);

	// convert the result to a list of agent IDs, possibly ignoring a certain agent
	for (size_t i = 0; i < nrResults; ++i)
	{
		const size_t pointCloudIndex = result_indices[i];
		const size_t agentID = pointCloud.agentPositions[pointCloudIndex].first;
		if (agentToIgnore != nullptr && agentID == agentToIgnore->getID())
			continue;
		result.push_back(agentID);

		// if we've reached k results now (which may be the case if agentToIgnore is not part of the result), ignore the farthest neighbor
		if (result.size() == k)
			break;
	}

	return result;
}