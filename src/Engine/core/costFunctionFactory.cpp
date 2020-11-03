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

#include <core/costFunctionFactory.h>

#include <CostFunctions/FOEAvoidance.h>
#include <CostFunctions/GenericCost.h>
#include <CostFunctions/GoalReachingForce.h>
#include <CostFunctions/Karamouzas.h>
#include <CostFunctions/Moussaid.h>
#include <CostFunctions/ORCA.h>
#include <CostFunctions/Paris.h>
#include <CostFunctions/PLEdestrians.h>
#include <CostFunctions/PowerLaw.h>
#include <CostFunctions/RandomFunction.h>
#include <CostFunctions/RVO.h>
#include <CostFunctions/SocialForcesAvoidance.h>
#include <CostFunctions/TtcaDca.h>
#include <CostFunctions/VanToll.h>

CostFunctionFactory::Registry CostFunctionFactory::registry = CostFunctionFactory::Registry();

void CostFunctionFactory::RegisterAllCostFunctions()
{
	// do the registration only once
	if (!registry.empty())
		return;

	registerCostFunction<FOEAvoidance>();
	registerCostFunction<GenericCost>();
	registerCostFunction<GoalReachingForce>();
	registerCostFunction<Karamouzas>();
	registerCostFunction<Moussaid>();
	registerCostFunction<ORCA>();
	registerCostFunction<PowerLaw>();
	registerCostFunction<RandomFunction>();
	registerCostFunction<SocialForcesAvoidance>();
	registerCostFunction<TtcaDca>();
	registerCostFunction<RVO>();
	registerCostFunction<Paris>();
	registerCostFunction<PLEdestrians>();
	registerCostFunction<VanToll>();
}

void CostFunctionFactory::ClearRegistry()
{
	registry.clear();
}