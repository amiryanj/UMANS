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

#ifndef COLOR_H
#define COLOR_H

/// A simple wrapper for a color with R, G, B, and A components between 0 and 255.
struct Color
{
	/// The red component of this Color (range 0-255).
	unsigned short r;
	/// The green component of this Color (range 0-255).
	unsigned short g;
	/// The blue component of this Color (range 0-255).
	unsigned short b;
	/// The alpha component of this Color (range 0-255).
	unsigned short a;

	/// Default constructor for Color; creates the color black.
	Color() : r(0), g(0), b(0), a(255) {}

	/// Creates a Color with the given R, G, B, and A components.
	Color(unsigned short r, unsigned short g, unsigned short b, unsigned short a=255) : r(r), g(g), b(b), a(a) {}

	/// <summary>Creates a Color based on HSV (Hue, Saturation, Value) values.</summary>
	/// <remarks>Source: https://gist.github.com/kuathadianto/.</remarks>
	/// <param name="H">The H (Hue) component of the color, between 0 and 360.</param>
	/// <param name="S">The S (Saturation) component of the color, between 0 and 1.</param>
	/// <param name="V">The V (Vaue) component of the color, between 0 and 1.</param>
	/// <returns>A Color with RGB values that constitute the same color as the given HSV values.</returns>
	static Color FromHSV(int H, double S, double V)
	{
		H = H % 360;

		double C = S * V;
		double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
		double m = V - C;
		double Rs, Gs, Bs;

		if (H < 60) // red --> yellow
		{ 
			Rs = C;
			Gs = X;
			Bs = 0;
		}
		else if (H < 120) // yellow --> green
		{ 
			Rs = X;
			Gs = C;
			Bs = 0;
		}
		else if (H < 180) // green --> cyan
		{ 
			Rs = 0;
			Gs = C;
			Bs = X;
		}
		else if (H < 240) // cyan --> blue
		{ 
			Rs = 0;
			Gs = X;
			Bs = C;
		}
		else if (H < 300) // blue --> magenta
		{ 
			Rs = X;
			Gs = 0;
			Bs = C;
		}
		else // magenta --> red
		{ 
			Rs = C;
			Gs = 0;
			Bs = X;
		}

		return Color(
			(int)((Rs + m) * 255),
			(int)((Gs + m) * 255),
			(int)((Bs + m) * 255));
	}
};

#endif //COLOR_H