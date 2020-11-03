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

#include <core/agent.h>
#include "ORCALine.h"
#include "stddef.h"

namespace ORCALibrary
{
	typedef std::vector<PhantomAgent> AgentNeighborList;

	/**
	* \brief       A sufficiently small positive number.
	*/
	const float EPSILON = 0.00001f;

	class Solver
	{
	public:
		void solveOrcaProgram(const Agent& agent,
			const float timeHorizon, const float currentTime, const float simulationTimeStep, const NeighborList& neighbors, const float maxDistance, Solution& result) const;

	private:
		/*void createObstacleOrcaLines(const Agent& agent, 
			std::vector<Line>& orcaLines_, const float timeHorizonObst_, const float simulationTimeStep, const ObstacleNeighborList& obstacleNeighbors_, const float maxDistance) const;*/
		void createAgentOrcaLines(const Agent& agent, 
			std::vector<Line>& orcaLines_, const float timeHorizon_, const float simulationTimeStep, const AgentNeighborList& agentNeighbors_, const float maxDistance) const;
	};

	/**
	* \relates    ORCASolver
	* \brief      Solves a one-dimensional linear program on a specified line
	*             subject to linear constraints defined by lines and a circular
	*             constraint.
	* \param      lines         Lines defining the linear constraints.
	* \param      lineNo        The specified line constraint.
	* \param      radius        The radius of the circular constraint.
	* \param      optVelocity   The optimization velocity.
	* \param      directionOpt  True if the direction should be optimized.
	* \param      result        A reference to the result of the linear program.
	* \return     True if successful.
	*/
	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
		float radius, const Vector2D &optVelocity,
		bool directionOpt, Vector2D &result);

	/**
	* \relates    ORCASolver
	* \brief      Solves a two-dimensional linear program subject to linear
	*             constraints defined by lines and a circular constraint.
	* \param      lines         Lines defining the linear constraints.
	* \param      radius        The radius of the circular constraint.
	* \param      optVelocity   The optimization velocity.
	* \param      directionOpt  True if the direction should be optimized.
	* \param      result        A reference to the result of the linear program.
	* \return     The number of the line it fails on, and the number of lines if successful.
	*/
	size_t linearProgram2(const std::vector<Line> &lines, float radius,
		const Vector2D &optVelocity, bool directionOpt,
		Vector2D &result);

	/**
	* \relates    ORCASolver
	* \brief      Solves a two-dimensional linear program subject to linear
	*             constraints defined by lines and a circular constraint.
	* \param      lines         Lines defining the linear constraints.
	* \param      numObstLines  Count of obstacle lines.
	* \param      beginLine     The line on which the 2-d linear program failed.
	* \param      radius        The radius of the circular constraint.
	* \param      result        A reference to the result of the linear program.
	*/
	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
		float radius, Vector2D &result);
}