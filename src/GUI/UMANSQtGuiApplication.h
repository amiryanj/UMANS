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

#ifndef UMANSQTGUIAPPLICATION_H
#define UMANSQTGUIAPPLICATION_H

#include <QtWidgets/QWidget>
#include "ui_UMANSQtGuiApplication.h"

class OGLWidget;

/// <summary>Wrapper for the UMANS GUI application.</summary>
class UMANSQtGuiApplication : public QWidget
{
	Q_OBJECT

private:
	Ui::Application ui;
	UMANSOpenGLWidget* simulationView;

public:
	UMANSQtGuiApplication(QWidget *parent = Q_NULLPTR);
	inline Ui::Application& GetUI() { return ui; }
};

#endif //UMANSQTGUIAPPLICATION_H