#include "PyAPI.h"
using namespace boost::python;


BOOST_PYTHON_MODULE(umans)
{
    class_<AgentData>("AgentData", init<>())
            .add_property("id", &AgentData::getId, &AgentData::setId)
            .add_property("position_x", &AgentData::getPosition_x, &AgentData::setPosition_x)
            .add_property("position_y", &AgentData::getPosition_y, &AgentData::setPosition_y)
            .add_property("velocity_x", &AgentData::getVelocity_x, &AgentData::setVelocity_x)
            .add_property("velocity_y", &AgentData::getVelocity_y, &AgentData::setVelocity_y)
            .add_property("viewingDirection_x", &AgentData::getViewingDirection_x, &AgentData::setViewingDirection_x)
            .add_property("viewingDirection_y", &AgentData::getViewingDirection_y, &AgentData::setViewingDirection_y)
    ;


    class_<UMANS>("UMANS", init<>())
//            .def("resizeAgentData", &UMANS::resizeAgentData)
            .def("startSimulation", &UMANS::startSimulation)

            .def("getSimulationTimeStep", &UMANS::getSimulationTimeStep)
            .def("doSimulationSteps", &UMANS::doSimulationSteps)
            .def("getAgentPositions", &UMANS::getAgentPositions)
            .def("setAgentPositions", &UMANS::setAgentPositions)

            .def("addAgent", &UMANS::addAgent)
            .def("removeAgent", &UMANS::removeAgent)

            .def("setAgentGoal", &UMANS::setAgentGoal)
            .def("cleanUp", &UMANS::cleanUp)

            .def("getNumberOfAgents", &UMANS::getNumberOfAgents)
            .def("getNumberOfObstacles", &UMANS::getNumberOfObstacles)
    ;
};

