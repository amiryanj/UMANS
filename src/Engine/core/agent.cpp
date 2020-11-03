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

#include <core/agent.h>
#include <core/worldBase.h>
#include "../3rd-party/ORCA/ORCASolver.h"

Agent::Agent(size_t id, const Agent::Settings& settings) :
	id_(id), settings_(settings),
	position_(0, 0),
	velocity_(0, 0),
	preferred_velocity_(0, 0),
	next_velocity_(0, 0),
	goal_(0, 0),
	velocity_without_forces_(0, 0),
	contact_forces_(0, 0),
	viewing_direction_(0, 0)
{
	// set the seed for random-number generation
	RNGengine_.seed((unsigned int)id);
}

#pragma region [Simulation-loop methods]

void Agent::ComputeNeighbors(WorldBase* world)
{
	// get the query radius
	float range = getPolicy()->getInteractionRange();

	// perform the query and store the result
	neighbors_ = world->ComputeNeighbors(position_, range, this);
}

void Agent::ComputePreferredVelocity()
{
	if (hasReachedGoal())
		preferred_velocity_ = Vector2D(0, 0);

	else
		preferred_velocity_ = (goal_ - position_).getnormalized() * getPreferredSpeed();
}

void Agent::ComputeNextVelocity(WorldBase* world)
{
	next_velocity_ = getPolicy()->ComputeNewVelocity(this, world);
}

void Agent::ComputeContactForces(WorldBase* world)
{
	contact_forces_ = getPolicy()->ComputeContactForces(this, world);
}

void Agent::UpdateVelocityAndPosition(float dt)
{
	// The concept of a "relaxation time" is already part of the policy. 
	// So, we may assume that next_velocity_ can be used immediately, without interpolation.
	velocity_without_forces_ = next_velocity_;

	// apply forces
	const float mass = 80.0f; // TODO: parameter?
	const auto& acceleration_forces = contact_forces_ / mass;
	velocity_ = velocity_without_forces_ + (acceleration_forces * dt);

	// update the viewing direction:
	// a) as a weighted average of the preferred and actual velocity
	const auto& vDiff = (2 * velocity_ + preferred_velocity_) / 3;
	if (vDiff.sqrMagnitude() > 0.01)
		viewing_direction_ = vDiff.getnormalized();
	// b) based on the velocity that ignores contact forces
	//if (velocity_without_forces_.sqrMagnitude() > 0.01)
	//	viewing_direction_ = velocity_without_forces_.getnormalized();

	// update the position
	position_ = position_ + (velocity_ * dt);
}

#pragma endregion

#pragma region [Advanced getters]

bool Agent::hasReachedGoal() const
{
	return (goal_ - position_).sqrMagnitude() <= getRadius() * getRadius();
}

#pragma endregion

#pragma region [Basic setters]

void Agent::setPosition(const Vector2D &position)
{
	position_ = position;
}

void Agent::setVelocity_ExternalApplication(const Vector2D &velocity, const Vector2D &viewingDirection)
{
	velocity_ = velocity;
	velocity_without_forces_ = velocity;
	viewing_direction_ = viewingDirection;
}

void Agent::setGoal(const Vector2D &goal)
{
	goal_ = goal;

	// look straight towards the goal
	if (goal_ != position_)
		viewing_direction_ = (goal_ - position_).getnormalized();
}

#pragma endregion

float Agent::ComputeRandomNumber(float min, float max)
{
	auto distribution = std::uniform_real_distribution<float>(min, max);
	return distribution(RNGengine_);
}