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

#ifndef LIB_POLICY_H
#define LIB_POLICY_H

#include <3rd-party/tinyxml/tinyxml2.h>
#include <core/costFunction.h>
#include <tools/vector2D.h>
#include <vector>
#include <memory>
#include <string>

class WorldBase;
class Agent;

/// <summary>A set of sampling parameters that can be used by a Policy.</summary>
struct SamplingParameters
{
	/// <summary>An enum describing the possible types of sampling: regular or random.</summary>
	enum class Type
	{
		REGULAR,
		RANDOM
	};
	static bool TypeFromString(const std::string &method, Type& result);

	/// <summary>An enum describing the possible base velocities for sampling, i.e. the center of the disk in which samples are taken.</summary>
	enum class Base
	{
		ZERO, CURRENT_VELOCITY
	};
	static bool BaseFromString(const std::string &method, Base& result);

	/// <summary>An enum describing the possible base directions for sampling, i.e. the 'neutral' direction around which samples are taken.</summary>
	enum class BaseDirection { UNIT, CURRENT_VELOCITY, PREFERRED_VELOCITY };
	static bool BaseDirectionFromString(const std::string &method, BaseDirection& result);

	/// <summary>An enum describing the possible radii for sampling, i.e. the radius of the disk in which samples are taken.</summary>
	enum class Radius { PREFERRED_SPEED, MAXIMUM_SPEED, MAXIMUM_ACCELERATION };
	static bool RadiusFromString(const std::string &method, Radius& result);

	// Default parameters for sampling:

	Type type = Type::REGULAR;
	Base base = Base::ZERO;
	BaseDirection baseDirection = BaseDirection::CURRENT_VELOCITY;
	Radius radius = Radius::PREFERRED_SPEED;
	float angle = 180;
	int speedSamples = 4;
	int angleSamples = 11;
	int randomSamples = 100;
	bool includeBaseAsSample = false;

	/// <summary>Creates a SamplingParameters object with the default settings for approximating global optimization.</summary>
	static SamplingParameters ApproximateGlobalOptimization()
	{
		SamplingParameters params;
		params.type = Type::REGULAR;
		params.base = Base::ZERO;
		params.baseDirection = BaseDirection::UNIT;
		params.radius = Radius::MAXIMUM_SPEED;
		params.angle = 360;
		params.angleSamples = 36;
		params.speedSamples = 11;
		params.includeBaseAsSample = true;
		return params;
	}
};

/// <summary>A navigation policy that agents can use for local navigation.</summary>
/// <remarks>In each frame of the simulation loop, an agent uses a policy to compute its next velocity to use. 
/// To do this, a policy contains the following main ingredients:<list>
/// <item>One or more *cost functions* that assign costs to velocities.</item>
/// <item>An *optimization method* that describes how to use these cost functions to compute a "best velocity".</item>
/// <item>Additional parameters that describe how to apply this "best velocity" to the agent.</item></list>
/// The UMANS library can reproduce many local navigation algorithms by making particular choices for each ingredient.</remarks>
class Policy
{
public:
	/// <summary>An enum describing the possible optimization methods of a Policy.</summary>
	enum class OptimizationMethod
	{
		/// <summary>Indicates that a Policy computes a "best velocity" by following the gradient of its cost functions.</summary>
		GRADIENT,
		/// <summary>Indicates that a Policy uses sampling to create several candidate velocities, 
		/// computes the cost of each candidate, and returns the candidate with the lowest cost.</summary>
		SAMPLING,
		/// <summary>Indicates that a Policy computes a "best velocity" by global optimization of its cost function.
		/// This option is useful for cost functions that have a closed-form solution for finding the optimum.</summary>
		GLOBAL
	};
	static bool OptimizationMethodFromString(const std::string &method, OptimizationMethod& result);

private:
	/// <summary>A weighted list of cost functions used by this Policy.</summary>
	CostFunctionList cost_functions_;
	/// <summary>The optimization method used by this Policy.</summary>
	OptimizationMethod optimizationMethod_ = OptimizationMethod::GRADIENT;
	/// <summary>The sampling parameters used by this Policy. Only used if the optimization method is OptimizationMethod::SAMPLING.</summary>
	SamplingParameters samplingParameters_;
	/// <summary>A "relaxation time" parameter used for interpolating between an agent's current and new velocity.
	/// The Policy itself only uses it if the optimization method is *not* OptimizationMethod::GRADIENT.
	/// In addition, specific cost functions may choose to use it (regardless of the optimization method).</summary>
	float relaxationTime_ = 0;
	/// <summary>A scaling factor to apply to contact forces. Use 0 to disable these forces completely.</summary>
	float contactForceScale_ = 5000;

public:
	/// <summary>Creates a Policy with the given details.</summary>
	/// <param name="method">The optimization method that this policy should use.</param>
	/// <param name="samplingParameters">The sampling parameters that this policy should use. 
	/// Only used if the optimization method is OptimizationMethod::SAMPLING.</param>
	Policy(OptimizationMethod method, SamplingParameters params)
		: optimizationMethod_(method), samplingParameters_(params) {}

	/// <summary>Destroys this Policy and all cost functions inside it.</summary>
	~Policy();

	/// <summary>Computes and returns a new velocity for a given agent, using the cost functions and optimization method of this Policy.</summary>
	/// <param name="agent">The agent for which a new velocity should be computed.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	Vector2D ComputeNewVelocity(Agent* agent, WorldBase* world);

	/// <summary>Computes and returns a 2D vector describing the contact forces experienced by a given agent due to collisions. 
	/// This force is already scaled by the scaling factor stored in this Policy.</summary>
	/// <param name="agent">The agent for which a new velocity should be computed.</param>
	/// <param name="world">The world in which the simulation takes place.</param>
	Vector2D ComputeContactForces(Agent* agent, WorldBase* world);

	/// <summary>Adds a cost function to this Policy's list of cost functions.</summary>
	/// <param name="costFunction">A pointer to an alraedy created cost function.</param>
	/// <param name="params">An XML object containing all parameters that the cost function may want to read.</param>
	void AddCostFunction(CostFunction* costFunction, const CostFunctionParameters& params);

	/// <summary>Returns the number of cost functions used by this Policy.</summary>
	size_t GetNumberOfCostFunctions() const { return cost_functions_.size(); }

	/// <summary>Finds and returns the range of interaction of this policy.</summary>
	/// <remarks>This is the largest "range" value among all cost functions in this policy.
	/// It is the radius (in meters) that the agent should use for its nearest-neighbor query.</remarks>
	float getInteractionRange() const;

	/// <summary>Sets this Policy's relaxation time to the given value.</summary>
	/// <param name="t">The desired new relaxation time. 
	/// Use 0 or less to let agents use their new velocity immediately.
	/// Use a higher value to let agents interpolate between their current and new velocity.</param>
	inline void setRelaxationTime(float t) { relaxationTime_ = t; }
	/// <summary>Returns the relaxation time of this Policy.</summary>
	inline float getRelaxationTime() const { return relaxationTime_; }
	/// <summary>Sets the scaling factor to apply to contact forces. Use 0 to ignore contact forces completely.</summary>
	inline void setContactForceScale(float s) { contactForceScale_ = s; }

private:
	/// <summary>Computes an acceleration vector for an agent by following the gradient of this Policy's cost functions.</summary>
	Vector2D getAccelerationFromGradient(Agent* agent, WorldBase* world);
	/// <summary> Computes the best new velocity for an agent by finding the global minimum of this Policy's cost functions. 
	/// If the cost function does not have a closed-form global optimum, or if the Policy has more than one cost function,
	/// this method will use sampling to *approximate* the solution.</summary>
	Vector2D getBestVelocityGlobal(Agent* agent, WorldBase* world);
	/// <summary>Computes the best velocity for an agent by approaching the global minimum of this Policy's cost function via sampling.</summary>
	Vector2D getBestVelocitySampling(Agent* agent, WorldBase* world, const SamplingParameters& params);
};


#endif //LIB_POLICY_H
