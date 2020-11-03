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

#ifndef UMANSOPENGLWIDGET_H
#define UMANSOPENGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLShaderProgram>

#include "VisualizationData.h"
#include <core/crowdSimulator.h>

class UMANSQtGuiApplication;

/// <summary>Contains the visualization and behavior of the UMANS GUI application. 
/// This class includes functions for drawing simulation details on the screen, 
/// and functions that respond to interaction with UI elements.</summary>
class UMANSOpenGLWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	UMANSOpenGLWidget(QWidget *parent = nullptr);
	~UMANSOpenGLWidget();

public slots:
	void PlaySimulation();
	void PauseSimulation();
	void ResetSimulation();
	void SetPlaybackMultiplier(int value);
	void OpenScenarioFileDialog();
	void ToggleCSVOutput();
	void ZoomToFit();
	void ToggleGrid();
	void ToggleScreenshots();
	void MakeScreenshot();

private slots:
	void updateSimulation();

protected:
	void initializeGL() override;
	void paintGL() override;
	void resizeGL(int width, int height) override;

	// events
	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
	void keyPressEvent(QKeyEvent *event) override;
	void keyReleaseEvent(QKeyEvent *event) override;

private:
	void showErrorMessage(const std::string& title, const std::string& details) const;
	void startNewSimulation(const std::string& scenarioFilename);
	void resetVisualization(bool deleteCurrentVisualization);

	Vector2D screenToWorld(const QPoint& screenPoint) const;
	QSizeF getScreenToWorldScale() const;

	void drawSimulation();
	void drawAgent(const Agent& agent);
	void drawEnvironment(const bool refresh = false);
	void drawGrid(const bool refresh = false);
	void addPointsToBuffer(const std::vector<Vector2D>& points, const QColor& color, const std::string& target, const double depth);
	void addSegmentsToBuffer(const std::vector<LineSegment2D>& segments, const QColor& color, const std::string& target, const double depth);

	void updateCursor();
	void updateSimulationTimerText();
	void setActiveAgent(Agent* agent);
	void checkActiveAgent();


private:
	std::map<std::string, VisualizationData*> visualizationData;

	QOpenGLShaderProgram program;
	QOpenGLFunctions *f;
	QTimer *simulationTimer;

	UMANSQtGuiApplication* mainApplication;

	CrowdSimulator* simulator;
	Agent* activeAgent;
	size_t activeAgentID;

	std::vector<std::string> settingsFiles;

	bool inAgentSelectMode;
	bool panning;
	bool simulationRunning;
	int playbackMultiplier;

	bool makeScreenshotsPerFrame;
	bool writeCSVOutput;

	/// <summary>The part of the world that is currently being shown in the OpenGL widget.</summary>
	QRectF viewBounds;

	/// <summary>The width and height of theis OpenGL widget, in pixels.</summary>
	QSizeF windowSize;

	/// <summary>The last screen position of the mouse.</summary>
	QPoint m_lastPos;

	int matrixLocationInShader;
	
};

#endif //UMANSOPENGLWIDGET_H
