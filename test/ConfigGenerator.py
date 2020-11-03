DeltaTime = 0.033
EndTime = 100

AgentRadius = 0.25
AgentPreferredSpeed = 1.3
AgentMaxSpeed = 1.6

costfunctions = ["GoalReaching", "TtcaDca", "RandomFunction"]
coeffs = [0.5, 1.5, 0.05]

NumAgentsPerGroup = 50
NumAgentsPerLane = 5
Spearation = 1
PositionGroup1 = [10,0.1]
PositionGroup2 = [-10,0]
GoalGroup1 = [-10,0]
GoalGroup2 = [10,0]

WORLDTABS = 0
AGENTSTABS = 1
AGENTTABS = 2
AGENTFIELDTABS = 3
POLICIESTABS = 1
POLICYTABS = 2
COSTFUNCTIONTABS = 3


def writeline(f, line):
    f.write(line)
    f.write("\n")

def writetabs(f, numtabs):
    for i in range(numtabs):
        f.write("    ")

def writeAgent(f, radius, prefspeed, maxspeed, pos, goal, policyid):
    writetabs(f, AGENTTABS)
    writeline(f, "<Agent rad=" + '"' + str(radius) + '"' + " pref_speed=" + '"' + str(prefspeed) + '"' + " max_speed=" + '"' + str(AgentMaxSpeed) + '"' + ">")
    writetabs(f, AGENTFIELDTABS)
    writeline(f, "<pos x=" + '"' + str(pos[0]) + '"' + " y=" + '"' + str(pos[1]) + '"' + "/>")
    writetabs(f, AGENTFIELDTABS)
    writeline(f, "<goal x=" + '"' + str(goal[0]) + '"' + " y=" + '"' + str(goal[1]) + '"' + "/>")
    writetabs(f, AGENTFIELDTABS)
    writeline(f, "<Policy id=" + '"' + str(policyid) + '"' + "/>")
    writetabs(f, AGENTTABS)
    writeline(f, "</Agent>")

def writeGroupOfAgents(f, agentspergroup, agentsperlane, separation, position, goalposition, AgentRadius, AgentPreferredSpeed, AgentMaxSpeed, policyid):
    for i in range(0, agentspergroup):
        posx = (i / agentsperlane)*separation + position[0]
        posy = (i % agentsperlane)*separation + position[1]
        goalx = (i / agentsperlane)*separation + goalposition[0]
        goaly = (i % agentsperlane)*separation + goalposition[1]
        writeAgent(f, AgentRadius, AgentPreferredSpeed, AgentMaxSpeed, [posx,posy], [goalx,goaly], policyid)


def writePolicy(f, policyid, costfunctions, coeffs):
    if(len(costfunctions) != len(coeffs)):
        raise Exception("Array length mismatch.")
    writetabs(f, POLICYTABS)
    writeline(f, "<Policy id=" + '"' + str(policyid) + '"' + " OptimizationMethod=\"gradient\">")
    for i in range(0, len(coeffs)):
        cf = costfunctions[i]
        c = coeffs[i]
        writetabs(f, COSTFUNCTIONTABS)
        writeline(f, "<costfunction name=" + '"' + str(cf) + '"' + " coeff=" + '"' + str(c) + '"' + "/>")
    writetabs(f, POLICYTABS)
    writeline(f, "</Policy>")
    
    
#Generate file
f = open("config.xml", "w+")
writeline(f, """<?xml version="1.0" encoding="utf-8"?>""")
writeline(f, """<Simulation delta_time=""" + '"' + str(DeltaTime) + '"' + """ end_time=""" + '"' + str(EndTime) + '"' + """/>""")

writeline(f, "<World>")
#policy
writetabs(f, POLICIESTABS)
writeline(f, "<Policies>")

writePolicy(f, 0, costfunctions, coeffs)

writetabs(f, POLICIESTABS)
writeline(f, "</Policies>")

#agents
writetabs(f, AGENTSTABS)
writeline(f, "<Agents>")

#lane1
writeGroupOfAgents(f, NumAgentsPerGroup, NumAgentsPerLane, Spearation, PositionGroup1, GoalGroup1, AgentRadius, AgentPreferredSpeed, AgentMaxSpeed, 0)

#lane2
writeGroupOfAgents(f, NumAgentsPerGroup, NumAgentsPerLane, Spearation, PositionGroup2, GoalGroup2, AgentRadius, AgentPreferredSpeed, AgentMaxSpeed, 0)

writetabs(f, AGENTSTABS)
writeline(f, "</Agents>")
writeline(f, "</World>")

f.close()
