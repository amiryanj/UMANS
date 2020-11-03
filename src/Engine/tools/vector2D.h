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

#ifndef LIB_VECTOR2D_H
#define LIB_VECTOR2D_H

#include <math.h>
#include <vector>

const double PI = 3.1415926535897;

/// <summary>A 2D vector, used for representing positions, velocities, etc.</summary>
class Vector2D
{
public:
	/// <summary>The x component of this vector.</summary>
	float x;
	/// <summary>The y component of this vector.</summary>
	float y;

	/// <summary>Creates a Vector2D with both components set to zero.</summary>
	Vector2D() : x(0.0f), y(0.0f) {}
	/// <summary>Creates a Vector2D with the given x and y components.</summary>
	/// <param name="x">The desired x component of the vector.</param>
	/// <param name="y">The desired y component of the vector.</param>
	Vector2D(float x, float y) : x(x), y(y) { }

	/// <summary>Computes and returns the magnitude of this Vector2D.</summary>
	/// <returns>The vector's magnitude, i.e. sqrt(x*x + y*y).</returns>
	inline float magnitude() const { return sqrtf(x * x + y * y); }

	/// <summary>Computes and returns the squared magnitude of this Vector2D.</summary>
	/// <returns>The vector's squared magnitude, i.e. x*x + y*y.</returns>
	inline float sqrMagnitude() const { return x * x + y * y; }

	/// <summary>Normalizes this Vector2D to unit length, by dividing both x and y by the magnitude.</summary>
	inline void normalize()
	{
		float mag = magnitude();
		if (mag > 0)
		{
			x /= mag;
			y /= mag;
		}
	}

	/// <summary>Computes and returns a Vector2D that is the normalized version of the current vector.</summary>
	/// <returns>A Vector2D with the same direction as the current vector, but with unit length.</returns>
	inline Vector2D getnormalized() const
	{
		Vector2D Result(x, y);
		Result.normalize();
		return Result;
	}

	/// <summary>Computes and returns the dot product of this Vector2D with another Vector2D.</summary>
	/// <param name="other">Another vector.</param>
	/// <returns>The dot product of the current Vector2D and 'other', i.e. x*other.x + y*other.y.</returns>
	inline float dot(const Vector2D& other) const
	{
		return x * other.x + y * other.y;
	}

	/// <summary>Adds another Vector2D to the current Vector2D.</summary>
	/// <param name="rhs">Another vector.</param>
	/// <returns>A reference to the current Vector2D after the operation has been performed.</returns>
	inline Vector2D& operator+=(const Vector2D& rhs)
	{
		x += rhs.x; y += rhs.y;
		return *this;
	}

	/// <summary>Subtracts another Vector2D to the current Vector2D.</summary>
	/// <param name="rhs">Another vector.</param>
	/// <returns>A reference to the current Vector2D after the operation has been performed.</returns>
	inline Vector2D& operator-=(const Vector2D& rhs)
	{
		x -= rhs.x; y -= rhs.y;
		return *this;
	}

	/// <summary>Checks and returns whether this Vector2D is the zero vector.</summary>
	/// <returns>true if both x and y are 0; false otherwise.</returns>
	inline bool isZero() const
	{
		return x == 0 && y == 0;
	}
};

typedef std::pair<Vector2D, Vector2D> LineSegment2D;

#pragma region [Arithmetic operators]

inline Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs)
{
	return Vector2D(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline Vector2D operator+(const Vector2D& lhs, const Vector2D& rhs)
{
	return Vector2D(lhs.x + rhs.x, lhs.y + rhs.y);
}

inline Vector2D operator*(float lhs, const Vector2D& rhs)
{
	return Vector2D(rhs.x * lhs, rhs.y * lhs);
}

inline Vector2D operator*(const Vector2D& lhs, float rhs)
{
	return rhs * lhs;
}

inline Vector2D operator/(const Vector2D& lhs, float rhs)
{
	return (1.0f / rhs) * lhs;
}

inline bool operator==(const Vector2D& p, const Vector2D& q)
{
	return p.x == q.x && p.y == q.y;
}

inline bool operator!=(const Vector2D& p, const Vector2D& q)
{
	return p.x != q.x || p.y != q.y;
}

inline Vector2D operator-(const Vector2D& lhs)
{
	return Vector2D(-lhs.x, -lhs.y);
}

inline float distance(const Vector2D& p, const Vector2D& q)
{
	return (p - q).magnitude();
}

inline float distanceSquared(const Vector2D& p, const Vector2D& q)
{
	return (p - q).sqrMagnitude();
}

#pragma endregion

#pragma region [Angle-related functions]

inline Vector2D rotateCounterClockwise(const Vector2D& v, float radians)
{
	const double Cos = cos(radians), Sin = sin(radians);
	return Vector2D((float)(Cos*v.x - Sin * v.y), (float)(Sin*v.x + Cos * v.y));
}

inline Vector2D rotateCounterClockwiseAroundPoint(const Vector2D& v, const Vector2D& pivot, float radians)
{
	return pivot + rotateCounterClockwise(v - pivot, radians);
}

inline float angle(const Vector2D& va, const Vector2D& vb)
{
	const double lengths = va.magnitude() * vb.magnitude();
	if (lengths == 0)
		return 0.0f;

	const double frac = (va.dot(vb) / lengths);

	// check if frac is out of range (this can happend due to numerical imprecision)
	if (frac < -1 || frac > 1)
		return 0.0f;

	return (float)acos(frac);
}

inline float cosAngle(const Vector2D& va, const Vector2D& vb)
{
	const double lengths = va.magnitude() * vb.magnitude();
	if (lengths == 0)
		return 0.0f;

	return (float)(va.dot(vb) / lengths);
}

inline bool isClockwise(const Vector2D &vector1, const Vector2D &vector2)
{
	float cross = vector1.x * vector2.y - vector1.y * vector2.x;
	return cross < 0;
}

inline bool isCounterClockwise(const Vector2D &vector1, const Vector2D &vector2)
{
	float cross = vector1.x * vector2.y - vector1.y * vector2.x;
	return cross > 0;
}

inline float counterClockwiseAngle(const Vector2D& va, const Vector2D& vb)
{
	float ang = angle(va, vb);
	if (isClockwise(va, vb))
		return -ang;
	return ang;
}

inline float clockwiseAngle(const Vector2D& va, const Vector2D& vb)
{
	float ang = angle(va, vb);
	if (isCounterClockwise(va, vb))
		return -ang;
	return ang;
}

inline bool isPointLeftOfLine(const Vector2D& p, const Vector2D& la, const Vector2D& lb)
{
	return isClockwise(p - la, lb - la);
}

inline bool isPointRightOfLine(const Vector2D& p, const Vector2D& la, const Vector2D& lb)
{
	return isCounterClockwise(p - la, lb - la);
}

#pragma endregion

inline Vector2D clampVector(const Vector2D& v, float maxLength)
{
	float mag = v.magnitude();
	if (mag <= maxLength)
		return v;
	return v.getnormalized() * maxLength;
}

inline bool getLineIntersection(const Vector2D& a, const Vector2D& b, const Vector2D& c, const Vector2D& d, Vector2D& result)
{
	// Line AB represented as a1x + b1y = c1 
	double a1 = b.y - a.y;
	double b1 = a.x - b.x;
	double c1 = a1 * a.x + b1 * a.y;

	// Line CD represented as a2x + b2y = c2 
	double a2 = d.y - c.y;
	double b2 = c.x - d.x;
	double c2 = a2 * c.x + b2 * c.y;

	double determinant = a1 * b2 - a2 * b1;

	if (fabs(determinant) < 0.00001) // The lines are parallel.
		return false;

	result = Vector2D(
		(float)((b2*c1 - b1 * c2) / determinant),
		(float)((a1*c2 - a2 * c1) / determinant)
	);

	return true;
}

inline Vector2D nearestPointOnLine(const Vector2D& pt, const Vector2D& la, const Vector2D& lb,
	bool onSegment = false, bool* resultIsOnSegment = nullptr, float* resultFrac = nullptr)
{
	if (la == lb)
		return la;

	Vector2D line(la - lb);
	float k = (pt - lb).dot(line) / line.sqrMagnitude();

	if (onSegment)
	{
		if (k > 1) k = 1;
		else if (k < 0) k = 0;
	}

	if (resultIsOnSegment != nullptr)
		*resultIsOnSegment = (k >= 0 && k <= 1);

	if (resultFrac != nullptr)
		*resultFrac = 1 - k;

	return (la * k) + (lb * (1 - k));
}

inline float distanceToLine(const Vector2D& pt, const Vector2D& la, const Vector2D& lb, bool onSegment = false)
{
	return distance(pt, nearestPointOnLine(pt, la, lb, onSegment));
}

inline float distanceToLineSquared(const Vector2D& pt, const Vector2D& la, const Vector2D& lb, bool onSegment = false)
{
	return distanceSquared(pt, nearestPointOnLine(pt, la, lb, onSegment));
}

inline std::vector<Vector2D> approximateDisk_Outline(const Vector2D& center, const float radius, int nrSegments = 16)
{
	float angDiff = (float)(2*PI / nrSegments);
	Vector2D rad(radius, 0);

	std::vector<Vector2D> corners;
	for (int i = 0; i < nrSegments; ++i)
	{
		float ang = i * angDiff;
		corners.push_back(center + rotateCounterClockwise(rad, ang));
	}

	return corners;
}

inline std::vector<Vector2D> approximateDisk_Triangles(const Vector2D& center, const float radius, int nrSegments = 16)
{
	const auto& corners = approximateDisk_Outline(center, radius, nrSegments);

	std::vector<Vector2D> res;
	for (int i = 0; i < nrSegments; ++i)
	{
		res.push_back(center);
		res.push_back(corners[i]);
		res.push_back(corners[(i + 1) % nrSegments]);
	}

	return res;
}

#endif // LIB_VECTOR2D_H
