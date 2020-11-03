# UMANS: Unified Microscopic Agent Navigation Simulator
# Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettré
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Contact: crowd_group@inria.fr
# Website: https://project.inria.fr/crowdscience/
# See the file AUTHORS.md for a list of all contributors.

cmake_minimum_required( VERSION 3.10 )
project( UMANS )
set (CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

#set(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

file( GLOB_RECURSE src_All src/*)
file( GLOB_RECURSE src_Engine src/Engine/* src/3rd-party/* )
file( GLOB_RECURSE src_Library src/Library/* )
file( GLOB_RECURSE src_ConsoleApplication src/ConsoleApplication/* )
file( GLOB_RECURSE src_GUI src/GUI/* )

# --- ensure that certain directories are recognized in "#include <...>" lines
include_directories( ./src/ ./src/Engine/ )

# === Include the GUI application (which requires Qt)?

option(UMANS_USE_GUI_APPLICATION "Include a GUI demo application (which relies on Qt)" OFF)
option(WITH_PYTHON3_API "Returns a python wrapper in form a shared library (umans.so/dll)" OFF)

# === Use multithreading?

option(UMANS_USE_MULTITHREADING "Allow the program to use multiple parallel threads" ON)
if (UMANS_USE_MULTITHREADING)
	if (MSVC)
	  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /openmp")
	else()
	  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
    endif ()
endif (UMANS_USE_MULTITHREADING)

# === Linux compatibility compiler option

if (NOT MSVC)
	add_definitions(-O3 -lstdc++fs -fPIC -shared)
endif() 

# === Engine project: a static library that performs the simulation

add_library(Engine STATIC ${src_Engine})

# === Console application

if (WIN32)
	add_executable(UMANS-ConsoleApplication-Windows ${src_ConsoleApplication})
	target_link_libraries(UMANS-ConsoleApplication-Windows Engine)
else()
	add_executable(UMANS-ConsoleApplication-Linux  ${src_ConsoleApplication})
	target_link_libraries(UMANS-ConsoleApplication-Linux Engine)
endif()

# === Dynamic library (e.g. a DLL on Windows) to include into other programs

add_library(UMANS-Library SHARED ${src_Library})
target_link_libraries(UMANS-Library Engine)

# === Qt demo project (if enabled)

if (UMANS_USE_GUI_APPLICATION)

	# necessary: gives the path to GUI src files to qt autogen files
	include_directories( ./src/GUI/ )

	# set autogen variables for qt
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	set(CMAKE_AUTOUIC ON)
	set(CMAKE_AUTOMOC ON)
	set(CMAKE_AUTORCC ON)
	
	# find relevant packages
	find_package(Qt5Core REQUIRED)
	find_package(Qt5Widgets REQUIRED)
	find_package(OpenGL REQUIRED)

	# --- include the Qt resources, so that the application icon gets shown
	set(UI_RESOURCES src/GUI/UMANSQtGuiApplication.qrc)
	qt5_add_resources(UI_RESOURCES_RCC ${UI_RESOURCES})
	
	# --- define the project itself
	add_executable(UMANS-GUI src/GUI/main.cpp ${src_GUI} ${UI_RESOURCES_RCC} ${UI_HEADERS})

	# --- the project depends on some libraries
	target_link_libraries(UMANS-GUI Engine Qt5::Core Qt5::Widgets ${OPENGL_LIBRARIES})

	# --- post-build command: copy the required Qt DLLs, to prevent runtime errors
	add_custom_command(TARGET UMANS-GUI POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Qt5::Core> $<TARGET_FILE_DIR:UMANS-GUI>
		COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Qt5::Gui> $<TARGET_FILE_DIR:UMANS-GUI>
		COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Qt5::Widgets> $<TARGET_FILE_DIR:UMANS-GUI>
		)

endif (UMANS_USE_GUI_APPLICATION)

# ========== Python API =============
if (WITH_PYTHON3_API)
    if(UNIX AND NOT APPLE)
        message("===== Compile with Python 3 =====")
        FIND_PACKAGE(Boost COMPONENTS python35)
        FIND_PACKAGE(PythonInterp 3)
        FIND_PACKAGE(PythonLibs 3 REQUIRED)
    else()
        FIND_PACKAGE(Boost COMPONENTS python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
        FIND_PACKAGE(PythonInterp 3)
        FIND_PACKAGE(PythonLibs 3 REQUIRED)
    endif()
    PYTHON_ADD_MODULE(umans src/Library/PyAPI.cpp src/Library/PyAPI.h
        ${src_Library} ${src_Engine}
        )
    TARGET_LINK_LIBRARIES(umans ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
endif(WITH_PY_API)
# =====================================

# === Clean up the project's directory structure

foreach(_source IN ITEMS ${src_All})
    get_filename_component(_source_path "${_source}" PATH)
    file(RELATIVE_PATH _source_path_rel "${CMAKE_CURRENT_SOURCE_DIR}/src/" "${_source_path}")
    string(REPLACE "/" "\\" _group_path "${_source_path_rel}")
    source_group("${_group_path}" FILES "${_source}")
endforeach()

