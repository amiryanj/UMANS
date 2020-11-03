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

#include <core/crowdSimulator.h>

#include <tools/HelperFunctions.h>
#include <tools/csvwriter.h>
#include <core/worldInfinite.h>
#include <core/worldToric.h>
#include <core/costFunctionFactory.h>

CrowdSimulator::CrowdSimulator()
{
	CostFunctionFactory::RegisterAllCostFunctions();
	writer_ = nullptr;
	end_time_ = MaxFloat;
}

void CrowdSimulator::StartCSVOutput(const std::string &dirname, bool flushImmediately)
{
	// create the CSV writer if it did not exist yet
	if (writer_ == nullptr)
		writer_ = new CSVWriter(flushImmediately);

	// try to set the directory
	if (!writer_->SetOutputDirectory(dirname))
	{
		std::cerr << "Error: Could not set CSV output directory to " << dirname << "." << std::endl
			<< "The program will be unable to write CSV output." << std::endl;
		delete writer_;
		writer_ = nullptr;
	}
}

void CrowdSimulator::StopCSVOutput()
{
	if (writer_ != nullptr)
	{
		delete writer_;
		writer_ = nullptr;
	}
}

CrowdSimulator::~CrowdSimulator()
{
	// delete the CSV writer?
	if (writer_ != nullptr)
		delete writer_;

	// delete all policies
	for (auto& policy : policies_)
		delete policy.second;
	policies_.clear();

	// clear all cost-function creation functions
	CostFunctionFactory::ClearRegistry();
}

void CrowdSimulator::RunSimulationSteps(int nrSteps)
{
	for (int i = 0; i < nrSteps; ++i)
	{
		world_->DoStep();

		if (writer_ != nullptr)
		{
			double t = world_->GetCurrentTime();
			const auto& agents = world_->GetAgents();

			std::map<int, Vector2D> poss;
			for (const Agent* agent : agents)
				poss[(int)agent->getID()] = agent->getPosition();

			writer_->AppendAgentPositions(poss, t);
		}
	}
}

void CrowdSimulator::RunSimulationUntilEnd(bool showProgressBar, bool measureTime)
{
	if (end_time_ == MaxFloat || end_time_ <= 0)
	{
		std::cerr << "Error: The end time of the simulation is not correctly specified." << std::endl
			<< "The simulation cannot be run because it is unclear when it should end." << std::endl;
		return;
	}

	const int nrIterations_ = (int)ceilf(end_time_ / world_->GetDeltaTime());

	// get the current system time; useful for time measurements later on
	const auto& startTime = HelperFunctions::GetCurrentTime();

	if (showProgressBar)
	{
		// do the simulation in K blocks, and show a progress bar with K blocks
		const int nrProgressBarBlocks = 20;
		int nrIterationsPerBlock = nrIterations_ / nrProgressBarBlocks;
		int nrIterationsDone = 0;

		// - print an empty progress bar first, to indicate how long the real bar will be
		std::string progressBar = "Simulation progress: |";
		for (int i = 0; i < nrProgressBarBlocks; ++i)
			progressBar += "-";
		progressBar += "|100%";
		std::cout << std::endl << progressBar << std::endl;

		// - on a new line, start writing a new progress bar
		std::cout << "                     [";
		for (int i = 0; i < nrProgressBarBlocks; ++i)
		{
			// run a block of iterations
			int nrIterationsToDo = (i+1 == nrProgressBarBlocks ? nrIterations_ - nrIterationsDone : nrIterationsPerBlock);
			RunSimulationSteps(nrIterationsToDo);

			// augment the progress bar
			std::cout << "#" << std::flush;
			nrIterationsDone += nrIterationsToDo;
		}
		std::cout << "]" << std::endl << std::endl;
	}
	else
	{
		// do all steps at once without any printing
		RunSimulationSteps(nrIterations_);
	}

	if (measureTime)
	{
		// report running times
		const auto& endTime = HelperFunctions::GetCurrentTime();
		const auto& timeSpent = HelperFunctions::GetIntervalMilliseconds(startTime, endTime);

		std::cout << "Time simulated: " << world_->GetCurrentTime() << " seconds." << std::endl;
		std::cout << "Computation time used: " << timeSpent / 1000 << " seconds." << std::endl;
	}

	if (writer_ != nullptr)
		writer_->Flush();
}

#pragma region [Loading a configuration file]

std::string getFolder(const std::string& filename)
{
	auto endOfPath = filename.find_last_of('/');
	return (endOfPath == std::string::npos ? "" : filename.substr(0, endOfPath + 1));
}

bool CrowdSimulator::FromConfigFile_loadWorld(const tinyxml2::XMLElement* worldElement)
{
	// load the world type
	WorldBase::Type worldType = WorldBase::Type::UNKNOWN_WORLD_TYPE;
	const char* type = worldElement->Attribute("type");
	if (type != nullptr)
		worldType = WorldBase::StringToWorldType(type);

	if (worldType == WorldBase::Type::UNKNOWN_WORLD_TYPE)
	{
		std::cerr << "Warning: No valid world type specified in the XML file. Selecting default type (Infinite)." << std::endl;
		worldType = WorldBase::Type::INFINITE_WORLD;
	}

	if (worldType == WorldBase::Type::INFINITE_WORLD)
	{
		world_ = std::make_unique<WorldInfinite>();
		std::cout << "Created Infinite world." << std::endl;
	}

	// for toric worlds, load the width and height
	else if (worldType == WorldBase::Type::TORIC_WORLD)
	{
		float width = -1, height = -1;
		worldElement->QueryFloatAttribute("width", &width);
		worldElement->QueryFloatAttribute("height", &height);

		if (width > 0 && height > 0)
		{
			world_ = std::make_unique<WorldToric>(width, height);
			std::cout << "Created Toric world, width " << width << " and height " << height << "." << std::endl;
		}
		else
		{
			std::cerr << "Error: No valid size specified for the toric world in the XML file." << std::endl
				<< "Make sure to specify a non-negative width and height." << std::endl;
			return false;
		}
	}

	return true;
}

bool CrowdSimulator::FromConfigFile_loadPoliciesBlock_ExternallyOrNot(const tinyxml2::XMLElement* xmlBlock, const std::string& fileFolder)
{
	// - if this block refers to another file, read it
	const char* externalFilename = xmlBlock->Attribute("file");
	if (externalFilename != nullptr)
	{
		tinyxml2::XMLDocument externalDoc;
		externalDoc.LoadFile((fileFolder + externalFilename).data());
		if (externalDoc.ErrorID() != 0)
		{
			std::cerr << "Could not load or parse Policies XML file at " << (fileFolder + externalFilename) << "." << std::endl
				<< "Please check this file location, or place your Policies in the main XML file itself." << std::endl;
			return false;
		}

		return FromConfigFile_loadPoliciesBlock(externalDoc.FirstChildElement("Policies"));
	}

	// - otherwise, read the agents straight from the file itself
	return FromConfigFile_loadPoliciesBlock(xmlBlock);
}

bool CrowdSimulator::FromConfigFile_loadAgentsBlock_ExternallyOrNot(const tinyxml2::XMLElement* xmlBlock, const std::string& fileFolder)
{
	// - if this block refers to another file, read it
	const char* externalFilename = xmlBlock->Attribute("file");
	if (externalFilename != nullptr)
	{
		tinyxml2::XMLDocument externalDoc;
		externalDoc.LoadFile((fileFolder + externalFilename).data());
		if (externalDoc.ErrorID() != 0)
		{
			std::cerr << "Could not load or parse Agents XML file at " << (fileFolder + externalFilename) << "." << std::endl
				<< "Please check this file location, or place your Agents in the main XML file itself." << std::endl;
			return false;
		}

		return FromConfigFile_loadAgentsBlock(externalDoc.FirstChildElement("Agents"));
	}

	// - otherwise, read the agents straight from the file itself
	return FromConfigFile_loadAgentsBlock(xmlBlock);
}

bool CrowdSimulator::FromConfigFile_loadObstaclesBlock_ExternallyOrNot(const tinyxml2::XMLElement* xmlBlock, const std::string& fileFolder)
{
	// - if this block refers to another file, read it
	const char* externalFilename = xmlBlock->Attribute("file");
	if (externalFilename != nullptr)
	{
		tinyxml2::XMLDocument externalDoc;
		externalDoc.LoadFile((fileFolder + externalFilename).data());
		if (externalDoc.ErrorID() != 0)
		{
			std::cerr << "Could not load or parse Obstacles XML file at " << (fileFolder + externalFilename) << "." << std::endl
				<< "Please check this file location, or place your Obstacles in the main XML file itself." << std::endl;
			return false;
		}

		return FromConfigFile_loadObstaclesBlock(externalDoc.FirstChildElement("Obstacles"));
	}

	// - otherwise, read the agents straight from the file itself
	return FromConfigFile_loadObstaclesBlock(xmlBlock);
}

bool CrowdSimulator::FromConfigFile_loadPoliciesBlock(const tinyxml2::XMLElement* xmlBlock)
{
	// load the elements one by one
	const tinyxml2::XMLElement* element = xmlBlock->FirstChildElement();
	while (element != nullptr)
	{
		// load a single element
		if (!FromConfigFile_loadSinglePolicy(element))
			return false;

		// go to the next element
		element = element->NextSiblingElement("Policy");
	}

	return true;
}

bool CrowdSimulator::FromConfigFile_loadAgentsBlock(const tinyxml2::XMLElement* xmlBlock)
{
	// load the elements one by one
	const tinyxml2::XMLElement* element = xmlBlock->FirstChildElement();
	while (element != nullptr)
	{
		// load a single element
		if (!FromConfigFile_loadSingleAgent(element))
			return false;

		// go to the next element
		element = element->NextSiblingElement("Agent");
	}

	return true;
}

bool CrowdSimulator::FromConfigFile_loadObstaclesBlock(const tinyxml2::XMLElement* xmlBlock)
{
	// load the elements one by one
	const tinyxml2::XMLElement* element = xmlBlock->FirstChildElement();
	while (element != nullptr)
	{
		// load a single element
		if (!FromConfigFile_loadSingleObstacle(element))
			return false;

		// go to the next element
		element = element->NextSiblingElement("Obstacle");
	}

	return true;
}

bool CrowdSimulator::FromConfigFile_loadSinglePolicy(const tinyxml2::XMLElement* policyElement)
{
	// --- Read mandatory parameters

	// Unique policy ID
	int policyID;
	policyElement->QueryIntAttribute("id", &policyID);

	// Optimization method
	auto methodName = policyElement->Attribute("OptimizationMethod");
	Policy::OptimizationMethod method;
	if (!Policy::OptimizationMethodFromString(methodName == nullptr ? "" : methodName, method))
	{
		std::cerr << "Error in Policy " << policyID << ": Optimization method invalid." << std::endl;
		return false;
	}

	// Sampling parameters
	SamplingParameters params;
	if (method == Policy::OptimizationMethod::SAMPLING)
	{
		policyElement->QueryAttribute("SamplingAngle", &params.angle);
		policyElement->QueryAttribute("SpeedSamples", &params.speedSamples);
		policyElement->QueryAttribute("AngleSamples", &params.angleSamples);
		policyElement->QueryAttribute("RandomSamples", &params.randomSamples);
		policyElement->QueryBoolAttribute("IncludeBaseAsSample", &params.includeBaseAsSample);

		// type of sampling (= random or regular)
		const char * res = policyElement->Attribute("SamplingType");
		if (res != nullptr && !SamplingParameters::TypeFromString(res, params.type))
			std::cerr << "Policy " << policyID << ": sampling type invalid, using default (regular)." << std::endl;

		// base velocity (= origin of the cone or circle being sampled)
		res = policyElement->Attribute("SamplingBase");
		if (res != nullptr && !SamplingParameters::BaseFromString(res, params.base))
			std::cerr << "Policy " << policyID << ": sampling base invalid, using default (zero)." << std::endl;

		// base direction (= central direction of the cone or circle)
		res = policyElement->Attribute("SamplingBaseDirection");
		if (res != nullptr && !SamplingParameters::BaseDirectionFromString(res, params.baseDirection))
			std::cerr << "Policy " << policyID << ": sampling base direction invalid, using default (current velocity)." << std::endl;

		// radius (= radius of the cone or circle)
		res = policyElement->Attribute("SamplingRadius");
		if (res != nullptr && !SamplingParameters::RadiusFromString(res, params.radius))
			std::cerr << "Policy " << policyID << ": sampling radius type invalid, using default (preferred speed)." << std::endl;
	}

	// --- Create the policy

	Policy* pl = new Policy(method, params);

	// --- Read optional parameters

	// Relaxation time
	float relaxationTime = 0;
	if (policyElement->QueryFloatAttribute("RelaxationTime", &relaxationTime) == tinyxml2::XMLError::XML_SUCCESS)
		pl->setRelaxationTime(relaxationTime);

	// Force scale
	float contactForceScale = 0;
	if (policyElement->QueryFloatAttribute("ContactForceScale", &contactForceScale) == tinyxml2::XMLError::XML_SUCCESS)
		pl->setContactForceScale(contactForceScale);

	// --- Read and create cost functions

	// read all cost functions that belong to the policy; instantiate them one by one
	auto* funcElement = policyElement->FirstChildElement("costfunction");
	while (funcElement != nullptr)
	{
		const auto& costFunctionName = funcElement->Attribute("name");
		CostFunction* costFunction = CostFunctionFactory::CreateCostFunction(costFunctionName);
		if (costFunction != nullptr)
			pl->AddCostFunction(costFunction, CostFunctionParameters(funcElement));

		funcElement = funcElement->NextSiblingElement();
	}

	if (pl->GetNumberOfCostFunctions() == 0)
	{
		std::cerr << "Error: Policy " << policyID << "needs at least one cost function element" << std::endl;
		delete pl;
		return false;
	}

	// --- Save the policy in the simulator

	if (!AddPolicy(policyID, pl))
	{
		std::cerr << "Error: Failed to add Policy " << policyID << " because its ID is already taken" << std::endl;
		delete pl;
		return false;
	}

	return true;
}

bool CrowdSimulator::FromConfigFile_loadSingleAgent(const tinyxml2::XMLElement* agentElement)
{
	// optional ID
	int agentID = -1;
	agentElement->QueryIntAttribute("id", &agentID);

	// optional agent parameters (if they are not provided, we use the default ones)
	Agent::Settings settings;
	agentElement->QueryFloatAttribute("rad", &settings.radius_);
	agentElement->QueryFloatAttribute("pref_speed", &settings.preferred_speed_);
	agentElement->QueryFloatAttribute("max_speed", &settings.max_speed_);
	agentElement->QueryFloatAttribute("max_acceleration", &settings.max_acceleration_);
	agentElement->QueryBoolAttribute("remove_at_goal", &settings.remove_at_goal_);

	// position
	float x, y;
	auto* positionElement = agentElement->FirstChildElement("pos");
	if (!positionElement)
	{
		std::cerr << "Error: Agent " << agentID << " needs a position element." << std::endl;
		return false;
	}
	positionElement->QueryFloatAttribute("x", &x);
	positionElement->QueryFloatAttribute("y", &y);
	Vector2D position(x, y);

	// policy
	auto* policyElement = agentElement->FirstChildElement("Policy");
	if (!policyElement)
	{
		std::cerr << "Error: Agent " << agentID << "needs a policy element." << std::endl;
		return false;
	}

	int agentPolicyID;
	policyElement->QueryIntAttribute("id", &agentPolicyID);
	auto policy = GetPolicy(agentPolicyID);
	if (policy == nullptr)
	{
		std::cerr << "Error: The policy with id " << agentPolicyID << " doesn't exist." << std::endl;
		return false;
	}

	settings.policy_ = policy;

	// optional start time
	float startTime = 0;
	agentElement->QueryFloatAttribute("start_time", &startTime);

	// --- Add the agent to the world.

	Agent* agent = world_->AddAgent(position, settings, (agentID >= 0 ? (size_t)agentID : std::numeric_limits<size_t>::max()), startTime);

	// optional goal
	Vector2D goal = position;
	auto* goalElement = agentElement->FirstChildElement("goal");
	if (!goalElement)
	{
		std::cerr << "Warning: Agent " << agentID << " has no goal element." << std::endl;
		std::cerr << "This agent will not move." << std::endl;
	}
	else
	{
		goalElement->QueryFloatAttribute("x", &x);
		goalElement->QueryFloatAttribute("y", &y);
		goal = Vector2D(x, y);
	}

	agent->setGoal(goal);
	return true;
}

bool CrowdSimulator::FromConfigFile_loadSingleObstacle(const tinyxml2::XMLElement* obstacleElement)
{
	std::vector<Vector2D> points;

	// load coordinates
	auto* pointElement = obstacleElement->FirstChildElement("Point");
	while (pointElement != nullptr)
	{
		float x, y;
		pointElement->QueryFloatAttribute("x", &x);
		pointElement->QueryFloatAttribute("y", &y);
		pointElement = pointElement->NextSiblingElement("Point");
		points.push_back(Vector2D(x, y));
	}

	// add obstacle to world
	world_->AddObstacle(points);
	return true;
}

CrowdSimulator* CrowdSimulator::FromConfigFile(const std::string& filename)
{
	// Parse the XML into the property tree.
	// If the path cannot be resolved, an exception is thrown.
	tinyxml2::XMLDocument doc;
	doc.LoadFile(filename.data());
	if (doc.ErrorID() != 0)
	{
		std::cerr << "Error: Could not load or parse XML file at " << filename << std::endl;
		return nullptr;
	}

	const std::string& fileFolder = getFolder(filename);

	// --- Check if this is a "main" config file that refers to another config file.

	tinyxml2::XMLElement* simConfigPathElement = doc.FirstChildElement("configPath");
	if (simConfigPathElement != nullptr)
	{
		// Location of the config file should be relative to the location of the *main* config file.
		// Check if the main config file lies in a subfolder.
		return CrowdSimulator::FromConfigFile(fileFolder + simConfigPathElement->Attribute("path"));
	}

	// --- Otherwise, we assume that this is a "regular" config file that contains the simulation itself.

	tinyxml2::XMLElement* simulationElement = doc.FirstChildElement("Simulation");
	if (simulationElement == nullptr)
	{
		std::cerr << "Error: No main Simulation element in the XML file" << std::endl;
		return nullptr;
	}

	CrowdSimulator* crowdsimulator = new CrowdSimulator();
	crowdsimulator->scenarioFilename_ = filename;

	//
	// --- Read the world parameters
	//

	tinyxml2::XMLElement* worldElement = simulationElement->FirstChildElement("World");
	if (worldElement == nullptr)
	{
		std::cerr << "Error: No World element in the XML file" << std::endl;
		delete crowdsimulator;
		return nullptr;
	}
	if (!crowdsimulator->FromConfigFile_loadWorld(worldElement))
	{
		std::cerr << "Error: Failed to load the world from the XML file" << std::endl;
		delete crowdsimulator;
		return nullptr;
	}

	//
	// --- Read the simulation parameters
	//

	// the length of a simulation step
	float delta_time = -1;
	simulationElement->QueryFloatAttribute("delta_time", &delta_time);
	if (delta_time <= 0)
	{
		std::cerr << "Error: No valid value for delta_time found in the XML file." << std::endl
			<< "This attribute of Simulation is mandatory and should be positive." << std::endl;
		delete crowdsimulator;
		return nullptr;
	}
	crowdsimulator->GetWorld()->SetDeltaTime(delta_time);

	// the total simulation time (optional)
	float end_time = -1;
	simulationElement->QueryFloatAttribute("end_time", &crowdsimulator->end_time_);

	//
	// --- Read policies
	//

	// read the block with all policies
	tinyxml2::XMLElement* policiesElement = simulationElement->FirstChildElement("Policies");
	if (policiesElement != nullptr && !crowdsimulator->FromConfigFile_loadPoliciesBlock_ExternallyOrNot(policiesElement, fileFolder))
	{
		std::cerr << "Error while loading policies. The simulation cannot be loaded." << std::endl;
		delete crowdsimulator;
		return nullptr;
	}

	// if there are no policies at this point, print an error, and stop loading
	if (!crowdsimulator->HasPolicies())
	{
		std::cerr
			<< "Error: Failed to load any policies for the simulation." << std::endl
			<< "A simulation needs a Policy block with at least one valid Policy element." << std::endl;
		delete crowdsimulator;
		return nullptr;
	}

	//
	// --- Read agents
	//

	// read the block with all agents
	tinyxml2::XMLElement* agentsElement = simulationElement->FirstChildElement("Agents");
	if (agentsElement != nullptr && !crowdsimulator->FromConfigFile_loadAgentsBlock_ExternallyOrNot(agentsElement, fileFolder))
	{
		std::cerr << "Error while loading agents. The simulation cannot be loaded." << std::endl;
		delete crowdsimulator;
		return nullptr;
	}

	// if there are no agents at this point, print a warning (but not an error, because an empty crowd is allowed)
	if (crowdsimulator->GetWorld()->GetAgents().empty())
	{
		std::cerr << "Warning: Failed to load any agents for the simulation." << std::endl
			<< "The simulation will start without agents." << std::endl;
	}

	// 
	// --- Read obstacles
	//

	// read the block with all obstacles
	tinyxml2::XMLElement* obstaclesElement = worldElement->FirstChildElement("Obstacles");
	if (obstaclesElement != nullptr && !crowdsimulator->FromConfigFile_loadObstaclesBlock_ExternallyOrNot(obstaclesElement, fileFolder))
	{
		std::cerr << "Error while loading obstacles. The simulation cannot be loaded." << std::endl;
		delete crowdsimulator;
		return nullptr;
	}

	return crowdsimulator;
}

#pragma endregion

bool CrowdSimulator::AddPolicy(int id, Policy* policy)
{
	// if a policy with this ID already existed, don't add the policy
	if (policies_.find(id) != policies_.end())
		return false;

	// store the policy
	policies_[id] = policy;
	return true;
}

Policy* CrowdSimulator::GetPolicy(int id)
{
	auto findIt = policies_.find(id);
	if (findIt == policies_.end())
		return nullptr;
	return findIt->second;
}