#include <core/crowdSimulator.h>
#include <algorithm>
#include <omp.h>
#include <string>
#include <clocale>

#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>

namespace py = boost::python;

/// <summary>A struct that describes the status of a single agent in the simulation.
/// This struct is used for communication between the UMANS library and external applications.</summary>
struct AgentData
{
    AgentData() {}
    /// The unique ID of the agent.
    int id;
    int getId() const { return id; }
    void setId(int value) { id = value; }

    /// The x coordinate of the agent at the current time.
    float position_x;
    float getPosition_x() const { return position_x; }
    void setPosition_x(float value) { position_x = value; }

    /// The y coordinate of the agent at the current time.
    float position_y;
    float getPosition_y() const { return position_y; }
    void setPosition_y(float value) { position_y = value; }

    /// The x component of the agent's velocity at the current time.
    float velocity_x;
    float getVelocity_x() const { return velocity_x; }
    void setVelocity_x(float value) { velocity_x = value; }

    /// The y component of the agent's velocity at the current time.
    float velocity_y;
    float getVelocity_y() const { return velocity_y; }
    void setVelocity_y(float value) { velocity_y = value; }

    /// The x component of the agent's viewing direction at the current time.
    float viewingDirection_x;
    float getViewingDirection_x() const { return viewingDirection_x; }
    void setViewingDirection_x(float value) { viewingDirection_x = value; }

    /// The y component of the agent's viewing direction at the current time.
    float viewingDirection_y;
    float getViewingDirection_y() const { return viewingDirection_y; }
    void setViewingDirection_y(float value) { viewingDirection_y = value; }
};


class UMANS
{
private:
    CrowdSimulator* cs;
    std::vector<AgentData> agentData;

public:
    UMANS() {}
    void resizeAgentData()
    {
        int agentDataSize = (size_t)pow(2, 1 + (int)ceil(log2(cs->GetWorld()->GetAgents().size())));
        this->agentData.reserve(agentDataSize);
    }

    bool startSimulation(std::string configFileName, int numberOfThreads=1)
    {
        std::setlocale(LC_NUMERIC, "en_US.UTF-8");

        omp_set_num_threads(numberOfThreads);

        // initialize a new crowd simulation; we'll fill it with the contents of the given config file
        cs = CrowdSimulator::FromConfigFile(configFileName);
        if (cs == nullptr)
        {
            return false;
        }

        cs->GetWorld()->SetNumberOfThreads(numberOfThreads);

        // prepare the agentData array
        resizeAgentData();

        return true;
    }

    float getSimulationTimeStep()
    {
        if (cs == nullptr)
            return -1;

        return cs->GetWorld()->GetDeltaTime();
    }

    bool doSimulationSteps(int nrSteps)
    {
        if (cs == nullptr)
            return false;

        cs->RunSimulationSteps(nrSteps);
        return true;
    }

    py::list getAgentPositions()
    {
        if (cs == nullptr)
            return py::list();

        const auto& agents = cs->GetWorld()->GetAgents();
        agentData.resize(agents.size());

        // check if the AgentData array is large enough; resize it if necessary
        if (agents.size() > agentData.size() || agents.size() * 4 < agentData.size())
            resizeAgentData();

        // fill the AgentData array with the current agent data
        #pragma omp parallel for
        for (int i = 0; i < (int)agents.size(); ++i)
        {
            agentData[i].id = (int)agents[i]->getID();
            agentData[i].position_x = agents[i]->getPosition().x;
            agentData[i].position_y = agents[i]->getPosition().y;
            agentData[i].velocity_x = agents[i]->getVelocity().x;
            agentData[i].velocity_y = agents[i]->getVelocity().y;
            agentData[i].viewingDirection_x = agents[i]->getViewingDirection().x;
            agentData[i].viewingDirection_y = agents[i]->getViewingDirection().y;
        }

        // convert the result to python::list
        py::list result;
        for (int i = 0; i < (int)agents.size(); ++i)
        {
            result.append(agentData[i]);
        }
        return result;
    }

    bool setAgentPositions(py::list& agentData_)
    {
        // convert the python list to std::vector
        std::vector<AgentData> agentData_vec(len(agentData_));
        for (int i = 0; i < len(agentData_); ++i)
            agentData_vec[i] = boost::python::extract<AgentData>(agentData_[i]);

#pragma omp parallel for
        for (int i = 0; i < agentData_vec.size(); ++i)
        {
            // find the agent with the given ID
            auto agent = cs->GetWorld()->GetAgent(agentData_vec[i].id);
            if (agent != nullptr)
            {
                Vector2D position(agentData_vec[i].position_x, agentData_vec[i].position_y);
                Vector2D velocity(agentData_vec[i].velocity_x, agentData_vec[i].velocity_y);
                Vector2D viewingDirection(agentData_vec[i].viewingDirection_x, agentData_vec[i].viewingDirection_y);

                // override the agent's position, velocity, and viewing direction
                agent->setPosition(position);
                agent->setVelocity_ExternalApplication(velocity, viewingDirection);
            }

        }
        return true;
    }

    int addAgent(float x, float y, float radius, float prefSpeed, float maxSpeed, float maxAcceleration, int policyID, int customID)
    {
        if (cs == nullptr)
            return -1;

        // fill in the agent's settings
        Agent::Settings settings;
        settings.radius_ = radius;
        settings.preferred_speed_ = prefSpeed;
        settings.max_speed_ = maxSpeed;
        settings.max_acceleration_ = maxAcceleration;
        settings.policy_ = cs->GetPolicy(policyID);

        // try to add an agent
        auto agent = cs->GetWorld()->AddAgent(
                    Vector2D(x, y),
                    settings,
                    (customID >= 0 ? customID : std::numeric_limits<size_t>::max()));

        if (agent != nullptr)
        {
            // set the agent's goal to its current position, so that it doesn't start moving
            agent->setGoal(agent->getPosition());

            // fill in the ID that the agent received
            return (int)agent->getID();
        }

        return -1;
    }

    bool removeAgent(int id)
    {
        if (cs == nullptr)
            return false;

        // try to remove the agent
        return cs->GetWorld()->RemoveAgent(id);
    }

    bool setAgentGoal(int id, float x, float y)
    {
        if (cs == nullptr)
            return false;

        // try to find the agent
        auto* agent = cs->GetWorld()->GetAgent(id);
        if (agent == nullptr)
            return false;

        agent->setGoal(Vector2D(x, y));
        return true;
    }

    bool cleanUp()
    {
        if (cs == nullptr)
            return false;

        delete cs;
        cs = nullptr;
//        agentData.clear();

        return true;
    }

    int getNumberOfAgents() {
        if (cs == nullptr)
            return 0;
        return (int)cs->GetWorld()->GetNumberOfAgents();
    }

    int getNumberOfObstacles() {
        if (cs == nullptr)
            return 0;
        return (int)cs->GetWorld()->GetNumberOfObstacles();
    }
};

