#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml, argparse
from importlib import import_module


#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,default='abb',help="List of camera names separated with commas")
args, _ = parser.parse_known_args()
robot_name=args.robot_name

sys.path.append('../toolbox')
from general_robotics_toolbox import Robot, q2R
#auto discovery
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()


####################Start Service and robot setup

robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)

state_w = robot_sub.SubscribeWire("robot_state")


print(robot.robot_info.device_info.device.name)

time.sleep(0.5)
robot_state_wire=state_w.TryGetInValue()
print("wire value set: ",robot_state_wire[0])
robot_state = robot_state_wire[1]
print("kin_chain_tcp: ", robot_state.kin_chain_tcp)
print("robot_joints: ", robot_state.joint_position)
position=robot_state.kin_chain_tcp[0]['position'] 
orientation=robot_state.kin_chain_tcp[0]['orientation'] 
print(position)
print(q2R(list(orientation)))

