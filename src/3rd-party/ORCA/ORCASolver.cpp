/*
 * =====================================
 * NOTICE: This is an adapted version of the file Agent.cpp from the RVO2 library.
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
 * Agent.cpp
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

#include "ORCASolver.h"
#include <core/worldBase.h>
#include <algorithm>

namespace ORCALibrary
{
	inline float det(const Vector2D &vector1, const Vector2D &vector2)
	{
		return (float)(vector1.x * vector2.y - vector1.y * vector2.x);
	}

	/* Search for the best new velocity. */
	void Solver::solveOrcaProgram(const Agent& agent, 
		const float timeHorizon, const float currentTime, const float simulationTimeStep, const NeighborList& neighbors, const float maxDistance,
		Solution& result) const
	{
		const Vector2D& prefVelocity_ = agent.getPreferredVelocity();
		const float maxSpeed_ = agent.getMaximumSpeed();

		// in RVO library's example:
		// sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
		// neighborDist = 15.0f;
		// maxNeighbors = 10;
		// timeHorizon = 5.0f;
		// timeHorizonObst = 5.0f;
		// radius = 2.0f;
		// maxSpeed = 2.0f;

		result = Solution();

		createAgentOrcaLines(agent, result.orcaLines, timeHorizon, simulationTimeStep, neighbors.first, maxDistance);

		// TODO: include obstacle lines
		// ...

		// solve linear program
		size_t lineFail = linearProgram2(result.orcaLines, maxSpeed_, prefVelocity_, false, result.velocity);
		result.isFeasible = lineFail == result.orcaLines.size(); 
		
		// if the linear program is not feasible, compute the "least bad" velocity via another linear program
		if (!result.isFeasible)
			linearProgram3(result.orcaLines, result.numObstLines, lineFail, maxSpeed_, result.velocity);
		
		// store the simulation time
		result.currentSimulationTime = currentTime;
	}

	void Solver::createAgentOrcaLines(const Agent& agent, std::vector<Line>& orcaLines_, const float timeHorizon_, const float simulationTimeStep,
		const AgentNeighborList& agentNeighbors_, const float maxDistance) const
	{
		const Vector2D& position_ = agent.getPosition();
		const Vector2D& velocity_ = agent.getVelocity();
		const float maxSpeed_ = agent.getMaximumSpeed();
		const float radius_ = agent.getRadius();
		const float radiusSq = radius_ * radius_;
		const float maxDistSq = maxDistance * maxDistance;

		/* Create agent ORCA lines. */
		for (const PhantomAgent& other : agentNeighbors_)
		{
			if (other.GetDistanceSquared() > maxDistSq)
				continue;

			const Vector2D& relativePosition = other.GetPosition() - position_;
			const Vector2D& relativeVelocity = velocity_ - other.GetVelocity();
			const float distSq = relativePosition.sqrMagnitude();
			const float combinedRadius = radius_ + other.realAgent->getRadius();
			const float combinedRadiusSq = combinedRadius * combinedRadius;

			Line line;
			Vector2D u;

			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2D& w = relativeVelocity - (relativePosition / timeHorizon_);
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = w.sqrMagnitude();

				const float dotProduct1 = w.dot(relativePosition);

				if (dotProduct1 < 0.0f && dotProduct1*dotProduct1 > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2D& unitW = w / wLength;

					line.direction = Vector2D(unitW.y, -unitW.x);
					u = unitW * (combinedRadius / timeHorizon_ - wLength);
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2D(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
					}
					else {
						/* Project on right leg. */
						line.direction = Vector2D(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) * -1 / distSq;
					}

					const float dotProduct2 = relativeVelocity.dot(line.direction);

					u = (line.direction * dotProduct2) - relativeVelocity;
				}
			}
			else {
				/* Collision. Project on cut-off circle of time timeStep. */

				/* Vector from cutoff center to relative velocity. */
				const Vector2D w = relativeVelocity - (relativePosition / simulationTimeStep);

				const float wLength = w.sqrMagnitude();
				const Vector2D unitW = w / wLength;

				line.direction = Vector2D(unitW.y, -unitW.x);
				u = unitW * (combinedRadius / simulationTimeStep - wLength);
			}

			line.point = velocity_ + (u * 0.5f);
			orcaLines_.push_back(line);
		}
	}

	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Vector2D &optVelocity, bool directionOpt, Vector2D &result)
	{
		const float dotProduct = lines[lineNo].point.dot(lines[lineNo].direction);
		const float discriminant = dotProduct*dotProduct + radius*radius - lines[lineNo].point.sqrMagnitude();

		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}

		const float sqrtDiscriminant = std::sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < lineNo; ++i) {
			const float denominator = det(lines[lineNo].direction, lines[i].direction);
			const float numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point);

			if (std::fabs(denominator) <= EPSILON) {
				/* Lines lineNo and i are (almost) parallel. */
				if (numerator < 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const float t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity.dot(lines[lineNo].direction) > 0.0f) {
				/* Take right extreme. */
				result = lines[lineNo].point + (lines[lineNo].direction * tRight);
			}
			else {
				/* Take left extreme. */
				result = lines[lineNo].point + (lines[lineNo].direction * tLeft);
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineNo].direction.dot(optVelocity - lines[lineNo].point);

			if (t < tLeft) {
				result = lines[lineNo].point + (lines[lineNo].direction * tLeft);
			}
			else if (t > tRight) {
				result = lines[lineNo].point + (lines[lineNo].direction * tRight);
			}
			else {
				result = lines[lineNo].point + (lines[lineNo].direction * t);
			}
		}

		return true;
	}

	size_t linearProgram2(const std::vector<Line> &lines, float radius, const Vector2D &optVelocity, bool directionOpt, Vector2D &result)
	{
		if (directionOpt) {
			/*
				* Optimize direction. Note that the optimization velocity is of unit
				* length in this case.
				*/
			result = optVelocity * radius;
		}
		else if (optVelocity.sqrMagnitude() > radius*radius) {
			/* Optimize closest point and outside circle. */
			result = optVelocity.getnormalized() * radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const Vector2D tempResult = result;

				if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return lines.size();
	}

	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine, float radius, Vector2D &result)
	{
		float distance = 0.0f;

		for (size_t i = beginLine; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > distance) {
				/* Result does not satisfy constraint of line i. */
				std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

				for (size_t j = numObstLines; j < i; ++j) {
					Line line;

					float determinant = det(lines[i].direction, lines[j].direction);

					if (std::fabs(determinant) <= EPSILON) {
						/* Line i and line j are parallel. */
						if (lines[i].direction.dot(lines[j].direction) > 0.0f) {
							/* Line i and line j point in the same direction. */
							continue;
						}
						else {
							/* Line i and line j point in opposite direction. */
							line.point = (lines[i].point + lines[j].point) * 0.5f;
						}
					}
					else {
						line.point = lines[i].point + lines[i].direction * (det(lines[j].direction, lines[i].point - lines[j].point) / determinant);
					}

					line.direction = (lines[j].direction - lines[i].direction).getnormalized();
					projLines.push_back(line);
				}

				const Vector2D tempResult = result;

				if (linearProgram2(projLines, radius, Vector2D(-lines[i].direction.y, lines[i].direction.x), true, result) < projLines.size()) {
					/* This should in principle not happen.  The result is by definition
						* already in the feasible region of this linear program. If it fails,
						* it is due to small floating point error, and the current result is
						* kept.
						*/
					result = tempResult;
				}

				distance = det(lines[i].direction, lines[i].point - result);
			}
		}
	}
}