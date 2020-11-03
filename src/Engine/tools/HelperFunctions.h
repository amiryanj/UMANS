/* UMANS: Unified Microscopic Agent Navigation Simulator
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettr�
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

#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <vector>
#include <string>
#include <chrono>

namespace HelperFunctions
{
	typedef std::chrono::time_point<std::chrono::high_resolution_clock> Timestamp;

	/// <summary>Returns the current system time.</summary>
	Timestamp GetCurrentTime();
	/// <summary>Returns the number of milliseconds between two time stamps.</summary>
	double GetIntervalMilliseconds(const Timestamp& startTime, const Timestamp& endTime);
	
	/// <summary>Splits a string into parts according to a given delimiter. 
	/// The delimiter itself will be excluded from these parts.</summary>
	/// <param name="str">The string to split.</param>
	/// <param name="delim">The delimiter to use for splitting.</param>
	/// <returns>A list of substrings, each containing a part of the input string between two subsequent delimiters.
	/// The delimiter itself is not included in any of these parts.</returns>
	std::vector<std::string> SplitString(const std::string& str, const char delim);

	/// <summary>Tries to convert a string to a double.</summary>
	double ParseDouble(const std::string& str);
	/// <summary>Tries to convert a string to an int.</summary>
	int ParseInt(const std::string& str);
	/// <summary>Tries to convert a string to a bool.</summary>
	bool ParseBool(const std::string& str);

	/// <summary>Converts an integer to a string.</summary>
	std::string ToString(int val);
	/// <summary>Converts an integer to a string with a given minimum length.
	/// If the integer is shorter than that, the string will be augmented with leading 0s.</summary>
	std::string ToStringWithLeadingZeros(int value, int length);

	template <typename T> inline T Clamp(T val, T minValue, T maxValue)
	{
		if (val < minValue)
			return minValue;

		if (val > maxValue)
			return maxValue;

		return val;
	}

	/// <summary>Checks and returns whether a given directory exists.</summary>
	/// <param name="dirname">The name of a directory, either absolute or relative.</param>
	/// <returns>true if the directory exists; false otherwise.</returns>
	bool DirectoryExists(const std::string& dirname);

	/// <summary>Tries to create a directory if it does not yet exist.</summary>
	/// <param name="dirname">The name of a directory, either absolute or relative.</param>
	/// <returns>true if the directory either was successfully created or already existed; false otherwise.</returns>
	bool CreateDirectoryIfNonExistent(const std::string& dirname);
};

#endif //HELPER_FUNCTIONS_H