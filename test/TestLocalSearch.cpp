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

#include "tools/localsearch.h"
#include "tools/vector2D.h"

#include <iostream>
using namespace std;


// (𝑥 − 2)^2  ==> 𝑥* = 2
double testfunc(Vector2D x, Vector2D* grad = nullptr)
{
    double y = pow(x.x() -2, 2);
    cout << "f(" << x.x() << ") = " << y << endl;
    if (grad)
    {
        double g = 2 * pow((x.x()-2), 1);
        cout << "grad(" << x.x() << ") = " << g << endl;
        grad->set(g, 0);
    }
    return y;
}

double f(Vector2D x)
{
    return testfunc(x, nullptr);
}

int main(int argc, char *argv[])
{
    Vector2D x0(0, 0);
    Vector2D g;

    int N_ITR = 5;
    for (int i=0; i<N_ITR; i++) {
        testfunc(x0, &g);
        double alfa = LocalSearch::backtr(x0, -g, f, 10, 1e-4, 0.5);
        x0 = x0 - alfa * g;
    }
    cout << "********** Result ***************\n" ;
    f(x0);
}
