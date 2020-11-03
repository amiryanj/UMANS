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

#include <core/costFunctionParameters.h>
#include <tools/vector2D.h>

bool CostFunctionParameters::ReadInt(const std::string &name, int &value) const
{
	if (xml->QueryAttribute(name.c_str(), &value) == tinyxml2::XML_SUCCESS)
		return true;
	return false;
}

bool CostFunctionParameters::ReadFloat(const std::string &name, float &value) const
{
	if (xml->QueryAttribute(name.c_str(), &value) == tinyxml2::XML_SUCCESS)
		return true;
	return false;
}

bool CostFunctionParameters::ReadBool(const std::string &name, bool &value) const
{
	if (xml->QueryAttribute(name.c_str(), &value) == tinyxml2::XML_SUCCESS)
		return true;
	return false;
}

bool CostFunctionParameters::ReadString(const std::string &name, std::string &value) const
{
	const char* str = xml->Attribute(name.c_str());
	if (str != nullptr)
	{
		value = std::string(str);
		delete[] str;
		return true;
	}
	return false;
}

bool CostFunctionParameters::ReadVector2D(const std::string &nameX, const std::string &nameY, Vector2D &value) const
{
	if (!ReadFloat(nameX, value.x))
		return false;
	if (!ReadFloat(nameY, value.y))
		return false;

	return true;
}