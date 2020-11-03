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

#ifndef LIB_MATRIX_H
#define LIB_MATRIX_H

#include <math.h>
#include <tools/vector2D.h>

/// <summary>A 2x2 rotation matrix. Used by certain navigation functions.</summary>
class Matrix
{
private:
	float a11_;
	float a12_;
	float a21_;
	float a22_;

public:
	/// <summary>Creates a Matrix object with all values set to zero.</summary>
	Matrix() 
		: a11_(0.0f), a12_(0.0f), a21_(0.0f), a22_(0.0f) {}

	/// <summary>Creates a Matrix object with the given values.</summary>
	/// <param name="a11">The desired top-left value of the matrix.</param>
	/// <param name="a12">The desired top-right value of the matrix.</param>
	/// <param name="a21">The desired bottom-left value of the matrix.</param>
	/// <param name="a22">The desired bottom-right value of the matrix.</param>
	Matrix(float a11, float a12, float a21, float a22) 
		: a11_(a11), a12_(a12), a21_(a21), a22_(a22) {} 

	/// <summary>Creates a Matrix object with its two columns filled in with the values from two given vectors.</summary>
	/// <param name="v1">A 2D vector containing the desired left column of the matrix.</param>
	/// <param name="v2">A 2D vector containing the desired right column of the matrix.</param>
	Matrix(const Vector2D& v1, const Vector2D& v2)
		: a11_(v1.x), a12_(v2.x), a21_(v1.y), a22_(v2.y) {}

	/// <summary>Computes and returns the transposed version of this Matrix.</summary>
	/// <returns>A Matrix object similar to the current one, but with a21 and a12 swapped.</return>
	inline Matrix GetTransposed() const
	{
		return Matrix(a11_, a21_, a12_, a22_);
	}

	/// <summary>Computes and returns the determinant of this Matrix.</summary>
	/// <returns>The determinant of this matrix: a11 * a22 - a21 * a12.</returns>
	inline float Determinant() const
	{
		return a11_ * a22_ - a21_ * a12_;
	}

	/// <summary>Returns the top-left component of this Matrix.</summary>
	inline float a11() const { return a11_; }
	/// <summary>Returns the top-right component of this Matrix.</summary>
	inline float a12() const { return a12_; }
	/// <summary>Returns the bottom-left component of this Matrix.</summary>
	inline float a21() const { return a21_; }
	/// <summary>Returns the bottom-right component of this Matrix.</summary>
	inline float a22() const { return a22_; }
};

inline Matrix operator+(const Matrix& lhs, const Matrix& rhs)
{
	return Matrix(lhs.a11() + rhs.a11(),
		lhs.a12() + rhs.a12(),
		lhs.a21() + rhs.a21(),
		lhs.a22() + rhs.a22());
}

inline Matrix operator-(const Matrix& lhs, const Matrix& rhs)
{
	return Matrix(lhs.a11() - rhs.a11(),
		lhs.a12() - rhs.a12(),
		lhs.a21() - rhs.a21(),
		lhs.a22() - rhs.a22());
}

inline Matrix operator*(const Matrix& lhs, const Matrix& rhs)
{
	return Matrix(lhs.a11()*rhs.a11() + lhs.a12() * rhs.a21(),
		lhs.a11()*rhs.a12() + lhs.a12() * rhs.a22(),
		lhs.a21()*rhs.a11() + lhs.a22() * rhs.a21(),
		lhs.a21()*rhs.a12() + lhs.a22() * rhs.a22());
}

inline Matrix operator*(const float& lhs, const Matrix& rhs)
{
	return Matrix(lhs*rhs.a11(), lhs*rhs.a12(), lhs*rhs.a21(), lhs*rhs.a22());
}

inline Matrix operator*(const Matrix& lhs, const float &rhs)
{
	return rhs * lhs;
}

inline Vector2D operator*(const Matrix& lhs, const Vector2D& rhs)
{
	return Vector2D(lhs.a11()*rhs.x + lhs.a12()*rhs.y, lhs.a21()*rhs.x + lhs.a22()*rhs.y);
}

inline Matrix outerProduct(const Vector2D& lhs, const Vector2D& rhs)
{
	return Matrix(lhs.x*rhs.x, lhs.x*rhs.y, lhs.y*rhs.x, lhs.y*rhs.y);
}

#endif // LIB_MATRIX_H
