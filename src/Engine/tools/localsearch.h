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

#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include <tools/vector2D.h>
#include <functional>

class LocalSearch
{
public:
    LocalSearch() {}
    static double backtr(const Vector2D& x0, const Vector2D& grad, const std::function<float(const Vector2D&)>& F, double alpha_0=1, double gamma=1e-4, double delta=0.5, double rhok=1e-8);
};

#endif // LOCALSEARCH_H
