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

#ifndef LIB_KARAMOUZAS_H
#define LIB_KARAMOUZAS_H

#include <core/costFunction.h>

/// @ingroup costfunctions
/// <summary>An implementation of the collision-avoidance cost function proposed by %Karamouzas & Overmars, 
/// in the paper "A velocity-based approach for simulating human collision avoidance" (2010).</summary>
/// <remarks>
/// ### Notes: 
/// <list>
/// <item>This cost function already includes a goal-reaching component.</item>
/// <item>The <tt>%Karamouzas.xml</tt> policy in our example files reproduces the original algorithm.</item>
/// </list>
/// 
/// ### Name in XML files:
/// <tt>%Karamouzas</tt>
///
/// ### Parameters: 
/// <list>
/// <item>alpha, beta, gamma, delta (float): Scalars for the four main components of the cost function.</item>
/// <item>t_max (float): The maximum time-to-collision to consider.</item>
/// </list>
/// </remarks>
class Karamouzas : public CostFunction
{
private:
	float alpha = 5;
	float beta = 0.5f;
	float gamma = 1;
	float delta = 1;
	float t_max = 8;

	// thresholds for time to collision
	const float tc_min = 2.5f;
	const float tc_mid = 6;
	const float tc_max = t_max;

	// thresholds for the max angular deviation
	const float d_min = 0.05f; // theoretically 0, but we don't want to accidentally ignore the preferred velocity due to numerical errors
	const float d_mid = (float)(PI / 6.0f);
	const float d_max = (float)(PI / 2.0f);

public:
	Karamouzas() : CostFunction() {}
	virtual ~Karamouzas() {}
	const static std::string GetName() { return "Karamouzas"; }

	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const override;
	void parseParameters(const CostFunctionParameters & params) override;

private:
	float getMaxDeviationAngle(const Agent* agent, const float ttc) const;
	float getMinSpeed(const Agent* agent, const float ttc) const;
	float getMaxSpeed(const Agent* agent, const float ttc) const;
};

#endif //LIB_KARAMOUZAS_H
