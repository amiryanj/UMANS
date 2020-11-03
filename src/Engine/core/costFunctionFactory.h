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

#ifndef _COST_FUNCTION_FACTORY_H
#define _COST_FUNCTION_FACTORY_H

#include <core/costFunction.h>
#include <functional>
#include <map>
#include <string>
#include <iostream>

/// <summary>A static class that can create new instances of cost functions upon request.</summary>
class CostFunctionFactory
{
public:
	typedef std::map<std::string, std::function<CostFunction*()>> Registry;

private:
	/// <summary>A mapping from names to CostFunction creators.</summary>
	static Registry registry;

private:
	/// <summary>Adds the given CostFunction class type to the registry, and stores it under the name specified in that class's GetName() function. 
	/// After completion of this method, CreateCostFunction() will be able to create instances of this class type.</summary>
	template<typename CostFunctionType> static void registerCostFunction()
	{
		const std::string& name = CostFunctionType::GetName();
		if (registry.count(name) > 0)
		{
			std::cerr << "Error: cost function " << name << " has already been registered." << std::endl;
			return;
		}
		registry[name] = [] { return new CostFunctionType(); };
	}

public:
	/// <summary>Registers all possible types of cost functions that can exist. 
	/// Call this method once before you start using CreateCostFunction().</summary>
	static void RegisterAllCostFunctions();

	/// <summary>Clears the list of registered cost functions. 
	/// Call this method when you destroy the Simulator.</summary>
	static void ClearRegistry();

	/// <summary>Creates and returns a new CostFunction instance whose name matches the given value.</summary>
	/// <remarks>This method looks through the registry created in RegisterAllCostFunctions().</remarks>
	/// <param name="name">The name of the cost function to create.</param>
	/// <returns>A pointer to a new instance of the CostFunction subclass whose GetName() matches the "name" parameter.</returns>
	static CostFunction* CreateCostFunction(const std::string& name)
	{
		if (registry.count(name) == 0)
		{
			std::cerr << "Error: cost function " << name << " does not exist. The following cost functions are known: ";
			for (auto &elm : registry)
				std::cerr << ", " << elm.first;
			std::cerr << "." << std::endl;
			return nullptr;
		}

		return registry[name]();
	}
};

#endif
