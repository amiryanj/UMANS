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

#include "UMANSQtGuiApplication.h"
#include <QtWidgets/QApplication>

void printBasicInfo()
{
	std::cout
		<< "-----------------------------------------------------" << std::endl
		<< "UMANS: Unified Microscopic Agent Navigation Simulator" << std::endl
		<< "Copyright(C) 2018 - 2020 Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettre" << std::endl
		<< "-----------------------------------------------------" << std::endl << std::endl;
}

int main(int argc, char *argv[])
{
	printBasicInfo(); 
	
	QApplication a(argc, argv);
	UMANSQtGuiApplication w;
	w.show();
	return a.exec();
}
