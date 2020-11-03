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

#ifndef VISUALIZATIONDATA_H
#define VISUALIZATIONDATA_H

#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

/// <summary>A struct that can contain data to be shown on the screen. 
/// Used by the UMANSOpenGLWidget class in the UMANS GUI application.</summary>
struct VisualizationData
{
private:

	QOpenGLFunctions* openGLFunctions;
	QOpenGLVertexArrayObject vao;
	QOpenGLBuffer vbo_positions, vbo_colors;

	GLenum _drawMode;

	std::vector<QVector3D> vertices;
	std::vector<QColor> colors;

	bool hasChanged;
	bool isEnabled;

public:

	VisualizationData(QOpenGLFunctions* f, GLenum drawMode, bool enabled=true) 
		: openGLFunctions(f), _drawMode(drawMode), isEnabled(enabled), hasChanged(false)
	{
		vao.create();
		ClearData();
	}

	~VisualizationData()
	{
		vbo_positions.destroy();
		vbo_colors.destroy();
		vao.destroy();
	}

	void ToggleEnabled()
	{
		isEnabled = !isEnabled;
	}

	void AddData(const std::vector<QVector3D>& newPoints, const std::vector<QColor>& newColors)
	{
		vertices.insert(vertices.end(), newPoints.begin(), newPoints.end());
		colors.insert(colors.end(), newColors.begin(), newColors.end());
		hasChanged = true;
	}

	void AddData(const QVector3D& newPoint, const QColor& newColor)
	{
		vertices.push_back(newPoint);
		colors.push_back(newColor);
		hasChanged = true;
	}

	void ClearData()
	{
		vertices.clear();
		colors.clear();
	}

	void Draw()
	{
		if (!isEnabled)
			return;
		
		// if the data has changed since last time, refill the buffers
		if (hasChanged)
		{
			fillBuffers();
			hasChanged = false;
		}

		// draw the vertex/color data from the buffers
		QOpenGLVertexArrayObject::Binder vaoBinder(&vao);
		glDrawArrays(_drawMode, 0, (GLsizei)vertices.size());
	}

private:

	template<typename T> void fillSingleBuffer(QOpenGLBuffer& vbo,
		int arrayIndex, const std::vector<T>& list,
		const std::function<float(const T&)>& f1,
		const std::function<float(const T&)>& f2,
		const std::function<float(const T&)>& f3)
	{
		// create the vertex buffer object, if necessary
		if (!vbo.isCreated())
			vbo.create();
		vbo.bind();

		const unsigned int elementSize = 3;
		GLfloat* data = new GLfloat[list.size() * 3];
		for (int i = 0; i < list.size(); ++i)
		{
			data[i * 3] = f1(list[i]);
			data[i * 3 + 1] = f2(list[i]);
			data[i * 3 + 2] = f3(list[i]);
		}
		vbo.allocate(data, (int)(list.size() * elementSize * sizeof(GLfloat)));

		delete[] data;

		// Store the vertex attribute bindings for the program.
		openGLFunctions->glEnableVertexAttribArray(arrayIndex);
		openGLFunctions->glVertexAttribPointer(arrayIndex, elementSize, GL_FLOAT, GL_FALSE, 0, 0);

		vbo.release();
	}

	void fillBuffers()
	{
		QOpenGLVertexArrayObject::Binder vaoBinder(&vao);

		// prepare the vertices
		fillSingleBuffer<QVector3D>(vbo_positions, 0, vertices, &QVector3D::x, &QVector3D::y, &QVector3D::z);

		// prepare the colors
		fillSingleBuffer<QColor>(vbo_colors, 1, colors, &QColor::redF, &QColor::greenF, &QColor::blueF);
	}

};

#endif //VISUALIZATIONDATA_H