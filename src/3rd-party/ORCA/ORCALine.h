/*
 * =====================================
 * NOTICE: This is an adapted version of the file Agent.h from the RVO2 library.
 * It has been changed by Wouter van Toll for integration into the UMANS library.
 *
 * Overview of changes:
 * - Renamed RVO(2) to ORCA.
 * - Removed all Agent details; these are now managed by our own Agent class.
 *   Only the ORCA-specific calculations remain.
 * - Moved the Line class to a separate file.
 * - Added the Solution struct.
 * - Changed the names of several functions.
 * - Some auxiliary classes and functions have different names in the UMANS library.
 *
 * Original RVO2 license header below:
 * =====================================
 *
 * Agent.h
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#pragma once

#include <tools/vector2D.h>

namespace ORCALibrary
{
	/**
	* \brief      Defines a directed line.
	*/
	class Line {
	public:
		/**
		* \brief     A point on the directed line.
		*/
		Vector2D point;

		/**
		* \brief     The direction of the directed line.
		*/
		Vector2D direction;
	};

	struct Solution
	{
		Vector2D velocity;
		bool isFeasible;
		std::vector<Line> orcaLines;
		size_t numObstLines = 0;
		float currentSimulationTime = -1;
	};
}