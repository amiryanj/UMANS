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

#ifndef LIB_POLYGON2D_H
#define LIB_POLYGON2D_H

#include <vector>
#include <cstddef>
#include <tools/vector2D.h>

/// <summary>A wrapper for a 2D polygon defined by a list of Vector2D points.</summary>
/// <remarks>Upon creation, a Polygon2D object pre-computes a list of edges *and* a triangulation.</remarks>
class Polygon2D
{
private:
	std::vector<Vector2D> vertices_;
	std::vector<LineSegment2D> edges_;
	std::vector<size_t> triangleIndices;

public:
	/// <summary>Creates a Polygon2D with the given vertices.</summary>
	Polygon2D(const std::vector<Vector2D>& vertices);

	/// <summary>Returns the vertices of this Polygon2D.</summary>
	inline const std::vector<Vector2D>& GetVertices() const { return vertices_; }
	/// <summary>Returns the edges of this Polygon2D.</summary>
	inline const std::vector<LineSegment2D>& GetEdges() const { return edges_; }

	/// <summary>Creates and returns a triangulation of this Polygon2D. 
	/// The triangle indices have been precomputed; this method only converts these triangles to actual objects.</summary>
	std::vector<std::vector<Vector2D>> GetTriangles() const;

private:
	void computeTriangulation();

};
#endif //LIB_POLYGON2D_H
