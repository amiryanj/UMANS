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

#ifndef LIB_CROWD_SIMULATOR_H
#define LIB_CROWD_SIMULATOR_H

#include <memory>
#include <core/worldBase.h>
#include <core/agent.h>
#include <fstream>
class CSVWriter;

/// <summary>Wrapper object that manages the overall crowd simulation.</summary>
class CrowdSimulator
{
private:

  /// <summary>The world in which the simulation takes place.</summary>
  std::unique_ptr<WorldBase> world_;

  /// <summary>A pointer to an optional CSVWriter that can write the simulation output to CSV files.</summary>
  CSVWriter* writer_;

  /// <summary>An optional time at which the simulation should end.
  /// Only used if this number is set in a configuration file.</summary>
  float end_time_;

  /// <summary>A list containing all navigation policies, ordered by ID.
  /// Each agent uses one of these policies for its navigation.</summary>
  std::map<int, Policy*> policies_;

  std::string scenarioFilename_;

public:

  /// <summary>Creates a new CrowdSimulator object by loading a given configuration file.</summary>
  /// <remarks>Note: The caller of this method is responsible for deleting the resulting CrowdSimulator object.</remarks>
  /// <param name="filename">The name of the configuration file to load.</param>
  /// <returns>A pointer to new CrowdSimulator object, or nullptr if the loading failed for any reason.</returns>
  static CrowdSimulator* FromConfigFile(const std::string& filename);
  
  /// <summary>Destroys this CrowdSimulator object.</summary>
  ~CrowdSimulator();

  const std::string& GetScenarioFilename() const { return scenarioFilename_; }

  /// <summary>Prepares this CrowdSimulator for writing simulation output (as CSV files) to the given directory.</summary>
  /// <param name="dirname">The name of the directory to use for output.</param>
  /// <param name="flushImmediately">Whether or not the CSV writer should write its output files as fast as possible. 
  /// If it is true, the output files will be updated after each simulation frame.
  /// If it is false, the data to write will be cached, and files will be written when the CrowdSimulator gets destroyed.</param>
  void StartCSVOutput(const std::string& dirname, bool flushImmediately);

  void StopCSVOutput();

  /// <summary>Runs the given number of simulation steps.</summary>
  /// <param name="nrSteps">The number of simulation steps to run; should be at least 1, otherwise nothing happens.</param>
  void RunSimulationSteps(int nrSteps=1);

  /// <summary>Runs the crowd simulation for the number of iterations specified in the previously loaded config file.</summary>
  /// <remarks>If the config file does not specify a number of iterations, then this method will do nothing.</remarks>
  /// <param name="showProgressBar">Whether or not to print a progress bar in the console.</param>
  /// <param name="measureTime">Whether or not to measure the total computation time and report it in the console.</param>
  void RunSimulationUntilEnd(bool showProgressBar, bool measureTime);
  
  /// <summary>Returns a pointer to the world in which the simulation takes place.</summary>
  WorldBase* GetWorld() { return world_.get(); }

  /// <summary>Stores a navigation policy inside the simulation, under the given ID.</summary>
  /// <param name="id">A unique ID for the policy.</param>
  /// <param name="policy">A Policy object.</param>
  /// <returns>true if the policy was succesfully added; false otherwise, i.e. if the ID was already taken.</returns>
  bool AddPolicy(int id, Policy* policy);

  /// <summary>Finds and returns the navigation policy with the given ID, or returns nullptr if it does not exist.</summary>
  /// <param name="id">The ID of the policy to find.</param>
  /// <returns>A pointer to the Policy stored under the given ID, or nullptr if no such Policy exists.</returns>
  Policy* GetPolicy(int id);

  inline bool HasPolicies() const { return !policies_.empty(); }

private:
	CrowdSimulator();

	bool FromConfigFile_loadWorld(const tinyxml2::XMLElement* worldElement);

	bool FromConfigFile_loadPoliciesBlock_ExternallyOrNot(const tinyxml2::XMLElement* policiesBlock, const std::string& fileFolder);
	bool FromConfigFile_loadPoliciesBlock(const tinyxml2::XMLElement* policiesBlock);
	bool FromConfigFile_loadSinglePolicy(const tinyxml2::XMLElement* policyElement);

	bool FromConfigFile_loadAgentsBlock_ExternallyOrNot(const tinyxml2::XMLElement* agentsBlock, const std::string& fileFolder);
	bool FromConfigFile_loadAgentsBlock(const tinyxml2::XMLElement* agentsBlock);
	bool FromConfigFile_loadSingleAgent(const tinyxml2::XMLElement* agentElement);

	bool FromConfigFile_loadObstaclesBlock_ExternallyOrNot(const tinyxml2::XMLElement* obstaclesBlock, const std::string& fileFolder);
	bool FromConfigFile_loadObstaclesBlock(const tinyxml2::XMLElement* obstaclesBlock);
	bool FromConfigFile_loadSingleObstacle(const tinyxml2::XMLElement* obstacleElement);
};

#endif
