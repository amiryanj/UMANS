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

//========================================================================
/*!
  @file		csvwriter.h
  @class	CSVWriter
  @date    	24/8/2018
  @brief
  @author  Javad Amirian, (C) 2018
*/
//========================================================================

#ifndef _CSV_WRITER_H
#define _CSV_WRITER_H

#include <tools/vector2D.h>

#include <string>
#include <vector>
#include <mutex>
#include <map>

/// <summary>A class for writing agent trajectories to CSV files in a specified folder.</summary>
class CSVWriter
{
private:
    struct TimeAndPos
    {
		double t_;
		float x_, y_;
        TimeAndPos(double t, float x, float y) {
            t_ = t; x_ = x; y_ = y;
        }
    };
    std::mutex mtx_;
    std::string dirname_;
    std::map<int, std::vector<TimeAndPos>> pos_log_;

	/// <summary>Whether or not this CSVWriter immediately calls Flush() at the end of each call to AppendAgentPositions().</summary>
	bool flushImmediately;

public:
	/// <summary>Creates a CSVWriter object.</summary>
	/// <param name="flushImmediately">Whether or not this CSVWriter should immediately call Flush() at the end of each call to AppendAgentPositions().</summary>
    CSVWriter(bool flushImmediately) : flushImmediately(flushImmediately) {}

	/// <summary>Tries to set the directory to which CSV output files will be written.
	/// If this directory does not yet exist, the program will try to create it, but this operation might fail.</summary>
	/// <param name="dirname">The path to the desired output directory.</param>
	/// <returns>true if the directory was successfully set (and created if necessary); 
	/// false otherwise, i.e. if the folder did not exist and could not be created for some reason.</returns>
    bool SetOutputDirectory(const std::string &dirname);

	/// <summary>Appends a set of agent positions to the output buffer. 
	/// Note: if the flushImmediately parameter is false, the result is not yet written to a file, and you need to call Flush() yourself.</summary>
	/// <param name="poss">A list of agent positions, ordered by agent ID in a map.</param>
	/// <param name="t">The current simulation time.</param>
    void AppendAgentPositions(std::map<int, Vector2D> &poss, double t);

	/// <summary>Writes all buffered output to CSV files, and then cleans the buffer.</summary>
	/// <remarks>Call this method whenever you have finished gathering trajectory data via AppendAgentPositions().</remarks>
	/// <returns>true if the output was successfully written; 
	/// false otherwise, e.g. if the output folder was never specified via SetOutputDirectory().</returns>
	bool Flush();

	/// <summary>Cleans up this CSVWriter for removal. This includes a final call to flush().</summary>
	~CSVWriter();
};

#endif // _CSV_WRITER_H
