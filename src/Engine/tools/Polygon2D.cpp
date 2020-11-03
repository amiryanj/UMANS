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

#include <tools/Polygon2D.h>

#include <array>
#include <3rd-party/earcut/earcut.hpp>

Polygon2D::Polygon2D(const std::vector<Vector2D>& vertices) 
	: vertices_(vertices)
{
	// compute the list of edges
	const size_t n = vertices.size();
	edges_.resize(n);
	for (size_t i = 0; i < vertices.size(); ++i)
		edges_[i] = { vertices[i], vertices[(i + 1) % n] };

	// compute a triangulation
	computeTriangulation();
}

#pragma region [Triangulation]

std::vector<std::vector<Vector2D>> Polygon2D::GetTriangles() const
{
	std::vector<std::vector<Vector2D>> result;
	for (size_t i = 0; i < triangleIndices.size(); i += 3)
		result.push_back({ vertices_[triangleIndices[i]], vertices_[triangleIndices[i + 1]], vertices_[triangleIndices[i + 2]] });
	return result;
}

void Polygon2D::computeTriangulation()
{
	using Point = std::array<double, 2>;

	// Fill polygon structure with actual data. Any winding order works.
	std::vector<Point> boundary;
	for (const auto& pt : vertices_)
		boundary.push_back({ pt.x, pt.y });
	std::vector<std::vector<Point>> polygon = { boundary };

	// Run tessellation
	// Returns array of indices that refer to the vertices of the input polygon.
	// Three subsequent indices form a triangle. Output triangles are clockwise.
	triangleIndices = mapbox::earcut<size_t>(polygon);
}

#pragma endregion