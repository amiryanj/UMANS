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

#ifndef LIB_WORLD_INFINITE_H
#define LIB_WORLD_INFINITE_H

#include <core/worldBase.h>

/// <summary>The simplest implementation of WorldBase: an unbounded plane with no wrap-around effects.</summary>
class WorldInfinite : public WorldBase
{
public:
	WorldInfinite();
	NeighborList ComputeNeighbors(const Vector2D& position, float search_radius, const Agent* queryingAgent) const override;
};

#endif //LIB_WORLD_INFINITE_H
