#description: GENERATES XML FILE for OCSR testing

from xml.etree.ElementTree import Element, SubElement, Comment
import xml.etree.ElementTree as ET
from xml.etree import ElementTree
from xml.dom import minidom
from enum import Enum
import math
import re
import Dispositioner



#DATA

#PREDEFINED CONFIGURATIONS
number_of_configurations = 9
class Configurations(Enum):
    CIRCLE_5 = 0
    CIRCLE_10 = 1
    CIRCLE_15 = 2
    CROSSING_1TO1_45 = 3
    CROSSING_1TO1 = 4
    CROSSING_1TO1_PAR = 5
    CROSSING_1TO10_45 = 6
    CROSSING_1TO10 = 7
    CROSSING_1TO10_PAR = 8
    CROSSING_25TO25_PAR = 9
    CROSSING_25TO25 = 10
    ONE_WAY_FLOW = 11
    EXAMPLE_GENERATED = 12
default_configuration = Configurations.CIRCLE_5

#PREDEFINED METHODS
class Methods(Enum):
    Helbing = 0
    PowerLaw = 1
    RVO = 2
    PLEdestrians = 3
    Karamouzas = 4
    Moussaid = 5
    ORCA = 6
    Dutra = 7


default_method = Methods.Helbing

# COMMON data
simulation_time = 0.1
radius = 0.3
pref_speed = 1.3
#OTHER data
max_speed = 1.6
neighbour_distance = 100

file_general_name = "" 

#MODEL SPECIFIC data
Helbing_sf_range = 5

RVO2_time_to_horizon = 2
RVO2_neighbours_distance = 5

PL_time_to_horizon = 3
PL_neighbours_distance = 10
PL_k = 1.5
PL_ksi = 0.54
PL_m = 2
PL_max_speed = 20


# ---------- useful functions -----------------
# ---------------------------------------------
# prettify xml format
def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    #rough_string = ElementTree.tostring(elem, 'utf-8')
    #reparsed = minidom.parseString(rough_string)

    reparsed = minidom.parseString(ElementTree.tostring(elem, 'utf-8'))

    return reparsed.toprettyxml(indent="  ")
# add a line to the beggining of a file
def line_prepender(filename, line):
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write(line.rstrip('\r\n') + '\n' + content)
# remove a line from the file filename at position line
def remove_line(filename, line_number):
    with open(filename, "r+") as f:
        succes = False
        d = f.readlines()
        if line_number <= (len(d) - 1) :
            success = True
            f.seek(0)
            iter_i = 0
            for i in d:
                if iter_i != line_number:
                    f.write(i)
                iter_i = iter_i+1
            f.truncate()
    return succes 


# ------------ main functions -----------------
# ---------------------------------------------
def generate_xml(basic_name,selected_configuration, selected_method, testing = False):

        #parameters        
        constant_velocities = True
        
        #name of the file
        if(testing):
            file_name = basic_name
        else:
            file_name = basic_name + selected_configuration.name +"_method_"+ selected_method.name + ".xml"



        #~~~~ XML FILE GENERATION ~~~  
        top = Element('World')
        #policies 
        Policies = SubElement(top, 'Policies')
        
        #0)Helbing: Social Forces
        if(selected_method == Methods.Helbing): 
            Policy = SubElement(Policies, 'Policy', {'id':"0",'OptimizationMethod':"gradient"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"SocialForces",'range':str(neighbour_distance)})
        
        #1)PowerLaw: 
        if (selected_method == Methods.PowerLaw): 
            Policy = SubElement(Policies, 'Policy', {'id':"0", 'OptimizationMethod':"gradient"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"PowerLaw",'range':str(neighbour_distance)})
        
        #2)RVO
        if (selected_method == Methods.RVO): 
            Policy = SubElement(Policies, 'Policy', {'id':"0", 'OptimizationMethod':"sampling", 'SamplingType':"random", 'SamplingBase':"current velocity", 'SamplingBaseDirection':"unit", 'SamplingAngle':"360", 'RandomSamples':"250", 'SamplingRadius':"maximum acceleration"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"RVO",'range':str(neighbour_distance)})
        
        #3)PLEdestrians
        if (selected_method == Methods.PLEdestrians): 
            Policy = SubElement(Policies, 'Policy', {'id':"0", 'OptimizationMethod':"sampling", 'SamplingType':"regular", 'SamplingBase':"zero", 'SamplingBaseDirection':"unit", 'SamplingAngle':"360", 'SpeedSamples':"4", 'AngleSamples':"36", 'SamplingRadius':"maximum speed"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"PLEdestrians",'t_min':str(1),'t_max':str(2),'range':str(neighbour_distance)})
        
        #4)Karamouzas
        if (selected_method == Methods.Karamouzas): 
            Policy = SubElement(Policies, 'Policy', {'id':"0", 'OptimizationMethod':"sampling", 'SamplingType':"regular", 'SamplingBase':"zero", 'SamplingBaseDirection':"preferred velocity", 'SamplingAngle':"180", 'SpeedSamples':"15", 'AngleSamples':"19", 'SamplingRadius':"maximum speed", 'RelaxationTime':"0.5"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"Karamouzas",'range':str(neighbour_distance)})
        
        #5)Moussaid
        if (selected_method == Methods.Moussaid): 
            Policy = SubElement(Policies, 'Policy', {'id':"0", 'OptimizationMethod':"sampling", 'SamplingType':"regular", 'SamplingBase':"zero", 'SamplingBaseDirection':"preferred velocity", 'SamplingAngle':"180", 'SpeedSamples':"5", 'AngleSamples':"19", 'SamplingRadius':"preferred speed", 'RelaxationTime':"0.5"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"Moussaid",'range':str(neighbour_distance)})  
        
        #6)ORCA
        if (selected_method == Methods.ORCA): 
            Policy = SubElement(Policies, 'Policy', {'id':"0",'OptimizationMethod':"global"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"ORCA",'range':str(neighbour_distance)})

        #7)Dutra
        if (selected_method == Methods.Dutra): 
            Policy = SubElement(Policies, 'Policy', {'id':"0", 'OptimizationMethod':"sampling", 'SamplingType':"regular", 'SamplingBase':"current velocity", 'SamplingBaseDirection':"unit", 'SamplingAngle':"360", 'SpeedSamples':"2", 'AngleSamples':"36", 'SamplingRadius':"maximum acceleration", 'RelaxationTime':"0.5"})
            costfunction = SubElement(Policy, 'costfunction', {'name':"TtcaDca", 'sigmaTtca':str(1),'sigmaDca':str(0.3), 'sigmaAngle_goal':str(2), 'sigmaSpeed_goal':str(2),'range':str(neighbour_distance)})           



        #Agents dispositions
        Agents = SubElement(top, 'Agents')

        # 1) CIRCLES 
        if(selected_configuration == Configurations.CIRCLE_5):
            agent_start = [[5,0], [1.5368,4.758], [-4.0553, 2.9248], [-4.0296,-2.9601], [1.5783, -4.7444]]
            agent_goal = [[-5,0], [-1.5368,-4.758], [4.0553, -2.9248], [4.0296,2.9601], [-1.5783, 4.7444]]
        if(selected_configuration == Configurations.CIRCLE_10):
            agent_start = [[5,0], [4.0425,2.9425], [1.5368,4.758], [-1.5575,4.7512], [-4.0553,2.9248], [-5, -0.0218], [-4.0296,-2.9601], [-1.516,-4.7646], [1.5783,-4.7444], [4.0681, -2.907] ]
            agent_goal =  [[-5,0], [-4.0425,-2.9425], [-1.5368,-4.758], [1.5575,-4.7512], [4.0553,-2.9248], [5,0.0218], [4.0296,2.9601], [1.516,4.7646], [-1.5783,4.7444], [-4.0681, 2.907] ]
        if(selected_configuration == Configurations.CIRCLE_15):
            agent_start = [[5,0], [4.5665,2.0363], [3.3413,3.7916], [1.5368,4.758], [-0.5342,4.9714], [-2.5126,4.3228], [-4.0533,2.9248], [-4.8949,1.0196], [-4.8858,-1.0623], [-4.0296,-2.9061], [-2.4747,-4.3346], [-0.4908,-4.9579], [1.5783,-4.7444], [3.3737,-3.6903], [4.5842,-1.9964] ]
            agent_goal =  [[-5,0], [-4.5665,-2.0363], [-3.3413,-3.7916], [-1.5368,-4.758], [0.5342,-4.9714], [2.5126,-4.3228], [4.0533,-2.9248], [4.8949,-1.0196], [4.8858,1.0623], [4.0296,2.9061], [2.4747,4.3346], [0.4908,4.9579], [-1.5783,4.7444], [-3.3737,3.6903], [-4.5842,1.9964] ]
        
        # 2) CROSSINGS 
        angle45 = (math.pi/4)
        angle225 = 5*(math.pi/4)
        cross_pos_x = 5*math.cos(angle45)
        cross_pos_y = 5*math.sin(angle45)
        cross_goal_x = 50000*math.cos(angle225)
        cross_goal_y = 50000*math.sin(angle225)
        if(selected_configuration == Configurations.CROSSING_1TO10_45):
                agent_start = [[7.788,-0.2296], [0.9082,-0.7006], [4.4009,0.0814], [4.5742,1.1261], [6.3771,1.3731], [2.4071,0.5284], [6.9514,-1.296], [2.5479,-0.8279], [3.4446,0.8416], [6.0217,-0.3397], [cross_pos_x, cross_pos_y] ]
                agent_goal =  [[-49992.212,-0.2296], [-49999.0918,-0.7006], [-49995.5991,0.0814], [-49995.5991,1.1261], [-49993.6229,1.3731], [-49997.5929,0.5284], [-49993.0486,-1.296], [-49997.4521,-0.8279], [-49996.5554,0.8416], [-49993.9783,-0.3397], [cross_goal_x,cross_goal_y] ]
        if(selected_configuration == Configurations.CROSSING_1TO10):
                agent_start = [[7.788,-0.2296], [0.9082,-0.7006], [4.4009,0.0814], [4.5742,1.1261], [6.3771,1.3731], [2.4071,0.5284], [6.9514,-1.296], [2.5479,-0.8279], [3.4446,0.8416], [6.0217,-0.3397], [0, 5] ]
                agent_goal =  [[-49992.212,-0.2296], [-49999.0918,-0.7006], [-49995.5991,0.0814], [-49995.5991,1.1261], [-49993.6229,1.3731], [-49997.5929,0.5284], [-49993.0486,-1.296], [-49997.4521,-0.8279], [-49996.5554,0.8416], [-49993.9783,-0.3397], [0,-50000] ]
        if(selected_configuration == Configurations.CROSSING_1TO10_PAR):
            agent_start = [[7.788,-0.2296], [0.9082,-0.7006], [4.4009,0.0814], [4.5742,1.1261], [6.3771,1.3731], [2.4071,0.5284], [6.9514,-1.296], [2.5479,-0.8279], [3.4446,0.8416], [6.0217,-0.3397], [-5, 0] ]
            agent_goal =  [[-49992.212,-0.2296], [-49999.0918,-0.7006], [-49995.5991,0.0814], [-49995.5991,1.1261], [-49993.6229,1.3731], [-49997.5929,0.5284], [-49993.0486,-1.296], [-49997.4521,-0.8279], [-49996.5554,0.8416], [-49993.9783,-0.3397], [50000,0] ]
        if(selected_configuration == Configurations.CROSSING_1TO1_45):
            agent_start = [[-5,0],  [cross_pos_x,cross_pos_y]]
            agent_goal =  [[5,0], [cross_goal_x,cross_goal_y]]
        if(selected_configuration == Configurations.CROSSING_1TO1):
            agent_start = [[-5,0],  [0,5]]
            agent_goal =  [[5,0], [0,-5]]
        if(selected_configuration == Configurations.CROSSING_1TO1_PAR):
            agent_start = [[-5,0],  [5,0]]
            agent_goal =  [[5,0], [-5,0]]

        #3) GENERATED
        if(selected_configuration == Configurations.CROSSING_25TO25_PAR):
            first_group_start = Dispositioner.generate_n_positions_in_range(25,[-11,-8],[-8,8],1)
            second_group_start = Dispositioner.generate_n_positions_in_range(25,[8,11],[-8,8],1)
            agent_start = first_group_start + second_group_start
            first_group_goal = Dispositioner.translate_positions(first_group_start, [50000, 0])
            second_group_goal = Dispositioner.translate_positions(second_group_start, [-50000, 0])
            agent_goal =  first_group_goal + second_group_goal
        if(selected_configuration == Configurations.CROSSING_25TO25):
            first_group_start = Dispositioner.generate_n_positions_in_range(25,[-11,-8],[-5,5],1)
            second_group_start = Dispositioner.generate_n_positions_in_range(25,[-5,5],[8,11],1)
            agent_start = first_group_start + second_group_start
            first_group_goal = Dispositioner.translate_positions(first_group_start, [50000, 0])
            second_group_goal = Dispositioner.translate_positions(second_group_start, [0, -50000])
            agent_goal =  first_group_goal + second_group_goal

        #3b) GENERATED with RANDOM VELOCITIES
        if(selected_configuration == Configurations.ONE_WAY_FLOW):
            print("here ")
            constant_velocities = False
            number_of_agents = 200
            base_velocity = 1.3
            vel_delta = 0.3
            min_dist = 0.5
            agent_start = Dispositioner.generate_n_positions_in_range(number_of_agents,[-25,-20],[-25,25],min_dist)         
            agent_goal =  Dispositioner.translate_positions(agent_start, [50000, 0])
            agent_speed = Dispositioner.generate_n_random_velocities_in_range(number_of_agents, [base_velocity - vel_delta, base_velocity + vel_delta])
            print("here and number of agent_speed :" + str(len(agent_speed)) )

        #4) DEFAULT GENERATED
        if(selected_configuration == Configurations.EXAMPLE_GENERATED):
            agent_start = Dispositioner.generate_n_positions_in_range(10,[-8,-5],[-8,8],1)
            first_group_goal = Dispositioner.translate_positions(first_group_start, [50000, 0])
            second_group_goal = Dispositioner.translate_positions(second_group_start, [-50000, 0])
            agent_goal =  first_group_goal + second_group_goal


        if(constant_velocities): 
            for ag_i in range(0,len(agent_start)):
                Agent = SubElement(Agents, 'Agent', {'rad':str(radius), 'pref_speed':str(pref_speed), 'max_speed':str(max_speed)})
                pos = SubElement(Agent, 'pos', {'x':str(agent_start[ag_i][0]),'y':str(agent_start[ag_i][1])})
                goal = SubElement(Agent, 'goal', {'x':str(agent_goal[ag_i][0]),'y':str(agent_goal[ag_i][1])})
                poly = SubElement(Agent, 'Policy', {'id':"0"})
        else: #one way flows
            print("here and number of agent_speed :" + str(len(agent_speed)) )
            for ag_i in range(0,len(agent_start)):
                Agent = SubElement(Agents, 'Agent', {'rad':str(radius), 'pref_speed':str(agent_speed[ag_i]), 'max_speed':str(max_speed)})
                pos = SubElement(Agent, 'pos', {'x':str(agent_start[ag_i][0]),'y':str(agent_start[ag_i][1])})
                goal = SubElement(Agent, 'goal', {'x':str(agent_goal[ag_i][0]),'y':str(agent_goal[ag_i][1])})
                poly = SubElement(Agent, 'Policy', {'id':"0"})

        #file generation
        with open(file_name, 'w') as fh1:
            fh1.write(prettify(top))
            fh1.close()

        #lines modifications 
        VersionName = '<?xml version="1.0" encoding="utf-8"?>'
        SubjectName = '<Simulation delta_time="'+ str(simulation_time) +'"/>'
        remove_line(file_name,0)
        line_prepender(file_name, VersionName + '\n' + SubjectName)

        #print results
        with open(file_name, 'r') as fh3:
            print(fh3.read())









#------------ MAIN ----------------------------

# ---------- generation examples --------------
# ---------------------------------------------

#0) TEST
generate_xml("test.xml",Configurations.CROSSING_1TO1, Methods.ORCA, True)



#1) ALL
#for config_i in range(0, 11):    
#    selected_configuration = Configurations(config_i)
#    for meth_i in range(0, 8):
#        selected_method = Methods(meth_i)
#        generate_xml(file_general_name,selected_configuration,selected_method)

#2)
#selected_configuration = Configurations.CROSSING_1TO10
#for meth_i in range(0, 7):
#    selected_method = Methods(meth_i)
#    generate_xml(file_general_name,selected_configuration,selected_method)        
    
#selected_configuration = Configurations.CROSSING_1TO10_PAR
#for meth_i in range(0, 7):
#    selected_method = Methods(meth_i)
#    generate_xml(file_general_name,selected_configuration,selected_method)   




#3) 3 METHODS
#ORCA,PowerLaw,RVO 1TO10
#selected_configuration = Configurations.CROSSING_1TO10

#selected_method = Methods.ORCA
#generate_xml(file_general_name,selected_configuration,selected_method)        
    
#selected_method = Methods.PowerLaw
#generate_xml(file_general_name,selected_configuration,selected_method)       

#selected_method = Methods.RVO
#generate_xml(file_general_name,selected_configuration,selected_method)    


#4) CROSSING
#selected_configuration = Configurations.CROSSING_1TO10
#for meth_i in range(0, 8):
#    selected_method = Methods(meth_i)
#    generate_xml(file_general_name,selected_configuration,selected_method)

#selected_configuration = Configurations.CROSSING_1TO10_PAR
#for meth_i in range(0, 8):
#    selected_method = Methods(meth_i)
#    generate_xml(file_general_name,selected_configuration,selected_method)


#5)
#generate_xml("ONE_WAY_FLOW_200_ORCA.xml",Configurations.ONE_WAY_FLOW, Methods.ORCA, True)
#generate_xml("ONE_WAY_FLOW_200_Karamouzas.xml",Configurations.ONE_WAY_FLOW, Methods.Karamouzas, True)
