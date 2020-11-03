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

#ifndef LIB_COST_FUNCTION_H
#define LIB_COST_FUNCTION_H

#include <core/costFunctionParameters.h>

#include <tools/vector2D.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <limits> // std::reference_wrapper

class WorldBase;
class Agent;
class CostFunction;
struct SamplingParameters;
struct PhantomAgent;

typedef std::vector<PhantomAgent> AgentNeighborList;
typedef std::vector<LineSegment2D> ObstacleNeighborList;
typedef std::pair<AgentNeighborList, ObstacleNeighborList> NeighborList;

typedef std::vector<std::pair<const CostFunction*, float>> CostFunctionList;

const float MaxFloat = std::numeric_limits<float>::max();

/// @defgroup costfunctions Cost functions
/// Implementations of specific cost functions for local navigation.

/// <summary>An abstract class for a cost function that agents use for their (local) navigation.</summary>
/// <remarks>To create a new cost function, you need to create a new class that inherits from CostFunction.
/// Your class should implement the following: 
/// <list type="bullet">
/// <item>const static std::string GetName(): This method should return the name that identifies the function in configuration files.</item>
/// <item>GetCost(): This method should compute and return the cost of a velocity according to your cost function.</item>
/// <item>(optionally) GetGradient(): This method should compute and return the gradient of your cost function at the given velocity.
/// If you do not implement it, the program will automatically approximate the gradient via sampling.</item>
/// <item>(optionally) parseParameters(): This method should parse any additional parameters that are specific to your cost function. 
/// Make sure to call the parent version of parseParameters() here, to parse any parameters that were already defined.</item>
/// </list>
/// Finally, to be able to use your cost function, add the following line to the file core/costFunctionFactory.cpp:
/// <code>registerCostFunction<MyNewFunction>();</code>
///
/// This will make sure that the program can dynamically create instances of your cost function when it is used in an XML file.
/// 
/// The GenericCost class is a template file that you can use for your own classes, if you wish.</remarks>

/**
### Example of implementing a subclass

Add a file CostFunctions/MyNewFunction.h:
  
  <code>
  #ifndef MY_NEW_FUNCTION_H
  #define MY_NEW_FUNCTION_H

  class MyNewFunction : public CostFunction {
  public:
    MyNewFunction() : CostFunction() {}
	virtual ~MyNewFunction() {}
	const static std::string GetName() { return "MyNewFunctionName"; }

	float GetCost(const Vector2D& velocity, const Agent* agent, const WorldBase * world) const override;

	//// optional
	Vector2D GetGradient(const Vector2D& velocity, const Agent* agent, const WorldBase * world) const override;

	//// optional
	void parseParameters(const CostFunctionParameters & params) override;
  };

  #endif //MY_NEW_FUNCTION_H
  </code>
  
Add a file CostFunctions/MyNewFunction.cpp:

  <code>
  #include <CostFunctions/MyNewFunction.h>

  float MyNewFunction::GetCost(const Vector2D& velocity, const Agent* agent, const WorldBase * world) const {
    //// Implementation
  }

  //// optional
  Vector2D MyNewFunction::GetGradient(const Vector2D& velocity, const Agent* agent, const WorldBase * world) const {
    //// Implementation
  }

  //// optional
  void MyNewFunction::parseParameters(const CostFunctionParameters & params) {
	CostFunction::parseParameters(params);
    //// Implementation
  }
  </code>

Add the following lines to core/costFunctionFactory.cpp:

  <code>
  #include <CostFunctions/MyNewFunction.h>
  //// ...
  registerCostFunction<MyNewFunction>();
  </code>

*/
class CostFunction
{
protected:
	/// <summary>The interaction range (in meters) for this cost function. </summary>
	/// <remarks>This is used as the search radius for nearest-neighbor queries.
	/// The value corresponds to the XML parameter "range".</remarks>
	float range_ = 5.0f;

public:
	CostFunction();
	virtual ~CostFunction();

	/// <summary>Returns the interaction range of this cost function.</summary>
	/// <remarks>This is the radius of the circle (around the active agent) in which the cost function operates.</remarks>
	inline float GetRange() const { return range_; }

#pragma region [Main operations]
	/// @name Main operations
	/// The core operations that each CostFunction should support.
	/// @{

	/// <summary>Computes the cost of a given velocity.</summary>
	/// <remarks>The cost can be thought of as the (un)attractiveness of the given velocity for the given agent. 
	/// A lower cost implies that the velocity is better for the agent.
	/// Note: Every (non-abstract) child class of CostFunction must implement this method.</remarks>
	/// <param name="velocity">The velocity for which the cost is requested.</param>
	/// <param name="agent">The agent that would use the requested velocity.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	/// <returns>A floating-point cost indicating the (un)attractiveness of the given velocity for the given agent.</param>
	virtual float GetCost(const Vector2D& velocity, Agent* agent, const WorldBase* world) const = 0;

	/// <summary>Computes the gradient of the cost function at a given velocity.</summary>
	/// <remarks>The gradient is a 2D vector that points in the direction of steepest ascent, i.e. the direction in which the cost increases the most. 
	/// By default, this method uses sampling to approximate the gradient.
	/// Subclasses of CostFunction may choose to implement something more specific (e.g. a closed-form gradient).</remarks>
	/// <param name="velocity">The velocity for which the gradient is requested.</param>
	/// <param name="agent">The agent that would use the requested velocity.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	/// <returns>A 2D vector denoting the gradient of the cost function at the given velocity.</param>
	virtual Vector2D GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase* world) const;

	/// <summary>Computes the global minimum of the cost function, i.e. the velocity with minimum cost.</summary>
	/// <remarks>By default, this method uses sampling to approximate the global minimum.
	/// Subclasses of CostFunction may choose to implement a better (e.g. closed-form) solution.</remarks>
	/// <param name="agent">The agent for which the optimal velocity is requested.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	/// <returns>The velocity with minimum cost (or an approximation thereof) for the given agent.</param>
	virtual Vector2D GetGlobalMinimum(Agent* agent, const WorldBase* world) const;

	/// @}
#pragma endregion

	/// <summary>Uses sampling to approximate the global minimum of a list of cost functions.
	/// <remarks>This method tries out several candidate velocities (sampled according to 'params'), 
	/// computes the total cost for each candidate (combining all functions in 'costFunctions'), 
	/// and returns the velocity with the lowest cost.</remarks>
	/// <param name="agent">The agent for which the optimal velocity is requested.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	/// <param name="params">Parameters for sampling the velocity space.</param>
	/// <param name="costFunctions">A list of cost functions to evaluate.</param>
	/// <returns>The sample velocity for which the sum of all cost-function values is lowest.</param>
	static Vector2D ApproximateGlobalMinimumBySampling(Agent* agent, const WorldBase* world, 
		const SamplingParameters& params, const CostFunctionList& costFunctions);

	/// <summary>Parses the parameters of the cost function.</summary>
	/// <remarks>By default, this method already loads the "range" parameter. 
	/// Override this method to let it load any additional parameters of your interest.</remarks>
	/// <param name="params">An XML block that contains all parameters of this cost function.</param>
	virtual void parseParameters(const CostFunctionParameters & params);

protected:

#pragma region [Helper methods for common cost-function tasks]
	/// @name Helper methods for common cost-function tasks
	/// Useful methods that can come in handy for many CostFunction implementations.
	/// @{

	/// <summary>Computes the expected time to collision of two disk-shaped objects with two hypothetical velocities.</summary>
	/// <param name="position1">The current position of object 1.</param>
	/// <param name="velocity1">The hypothetical velocity of object 1.</param>
	/// <param name="radius1">The radius (in meters) of object 1.</param>
	/// <param name="position2">The current position of object 2.</param>
	/// <param name="velocity2">The hypothetical velocity of object 2.</param>
	/// <param name="radius2">The radius (in meters) of object 2.</param>
	/// <returns>The time (in seconds) after which the two objects will collide if they move at the given velocities. 
	/// This value is 0 if the objects are already colliding, and it is MaxFloat if the objects will never collide.</returns>
	float ComputeTimeToCollision(
		const Vector2D& position1, const Vector2D& velocity1, const float radius1,
		const Vector2D& position2, const Vector2D& velocity2, const float radius2) const;

	/// <summary>Computes the expected time to the first collision with a set of neighboring agents and obstacles.</summary>
	/// <param name="position">The current position of the querying agent.</param>
	/// <param name="velocity">The hypothetical velocity of the querying agent.</param>
	/// <param name="radius">The radius (in meters) of the querying agent.</param>
	/// <param name="neighbors">A list of neighboring agents and obstacles.</param>
	/// <param name="maximumDistance">The maximum distance between the query position and a neighboring object.
	/// Any objects farther away will be ignored.</param>
	/// <param name="ignoreCurrentCollisions">Whether or not to ignore any collisions that are already happening now.</param>
	/// <returns>The smallest time to collision (in seconds) of the querying agent with all neighbors in the list. 
	/// This value may be 0 if there is already a collision with any of the neighboring objects.
	/// However, if ignoreCurrentCollisions is set to true, any neighbors that are already colliding will be ignored, 
	/// and the result will be larger than zero.</returns>
	/// <seealso cref="ComputeTimeToCollision"/>
	float ComputeTimeToFirstCollision(
		const Vector2D& position, const Vector2D& velocity, const float radius,
		const NeighborList& neighbors, float maximumDistance, bool ignoreCurrentCollisions) const;

	/// <summary>Computes the expected time at which the distance between two disk-shaped objects is minimal, and the value of this distance.</summary>
	/// <param name="position1">The current position of object 1.</param>
	/// <param name="velocity1">The hypothetical velocity of object 1.</param>
	/// <param name="radius1">The radius (in meters) of object 1.</param>
	/// <param name="position2">The current position of object 2.</param>
	/// <param name="velocity2">The hypothetical velocity of object 2.</param>
	/// <param name="radius2">The radius (in meters) of object 2.</param>
	/// <returns>A pair of two floating-point numbers, where the first is the time to closest approach (ttca) 
	/// and the second is the distance of closest approach (dca).</returns>
	std::pair<float, float> ComputeTimeAndDistanceToClosestApproach(
		const Vector2D& position1, const Vector2D& velocity1, const float radius1,
		const Vector2D& position2, const Vector2D& velocity2, const float radius2) const;

	/// <summary>Converts a gradient in the "(angle,speed) domain" to a gradient in the Euclidean (vx,vy) domain.</summary>
	/// <param name="GradTh">The angular component of the gradient to convert.</param>
	/// <param name="GradS">The speed component of the gradient to convert.</param>
	/// <param name="direction">The directional vector that defines the basis of the angle-speed coordinate system.</param>
	/// <param name="speed">The speed that defines the scale of the angle-speed coordinate system.</param>
	/// <returns>A Vector2D that represents the same gradient in the (vx,vy) domain.</returns>
	Vector2D RotateGradientToEuclideanCoordinates(
		const float GradTh, const float GradS, const Vector2D& direction, const float speed) const;

	/// @}
#pragma endregion

private:
	/// <summary>Solves (for x) the quadratic equation ax^2 + bx + c = 0.</summary>
	/// <param name="a">The parameter a of the quadratic equation.</param>
	/// <param name="b">The parameter b of the quadratic equation.</param>
	/// <param name="c">The parameter c of the quadratic equation.</param>
	/// <param name="answer1">[out] Will store the first solution of the equation, if it exists. 
	/// This will only receive a value if there are 1 or 2 solutions.</param>
	/// <param name="answer2">[out] Will store the second solution of the equation, if it exists. 
	/// This will only receive a value if there are 2 solutions.</param>
	/// <returns>The number of solutions to the equation, i.e. 0, 1, or 2.</returns>
	int SolveQuadraticEquation(float a, float b, float c, float& answer1, float& answer2) const;

	/// <summary>Computes the expected time to collision of a moving disk-shaped object and a static line segment.</summary>
	/// <param name="position">The current position of the moving object.</param>
	/// <param name="velocity">The hypothetical velocity of the moving object.</param>
	/// <param name="radius">The radius (in meters) of the moving object.</param>
	/// <param name="lineSegment">The static line-segment object.</param>
	/// <returns>The time (in seconds) after which the two objects will collide if the disk moves at the given velocity.
	/// This value is 0 if the objects are already colliding, and it is MaxFloat if the objects will never collide.</returns>
	float ComputeTimeToCollision_LineSegment(const Vector2D& position, const Vector2D& velocity, const float radius,
		const LineSegment2D& lineSegment) const;

	float ComputeTimeToCollision_LineSegmentInterior(const Vector2D& position, const Vector2D& velocity, const float radius,
		const LineSegment2D& lineSegment) const;
};

#endif //LIB_COST_FUNCTION_H
