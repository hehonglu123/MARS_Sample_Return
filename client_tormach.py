#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module
from general_robotics_toolbox import *


def fwd(q):	
	global robot_def
	return fwdkin(robot_def,q)

def inv(p,R=np.array([[0,0,1],[0,1,0],[-1,0,0]])):
	global robot_def
	pose=Transform(R,p)
	q_all=robot6_sphericalwrist_invkin(robot_def,pose)
	###TODO: find one q
	return q

def jacobian(q):
	global robot_def
	return robotjacobian(robot_def,q)

def main():
	global robot_def
	####################Start Service and robot setup

	robot_sub=RRN.SubscribeService('rr+tcp://pathpilot:11111?service=tormach_robot')
	robot=robot_sub.GetDefaultClientWait(1)
	state_w = robot_sub.SubscribeWire("robot_state")

	print(robot.robot_info.device_info.device.name+" Connected")

	num_joints=len(robot.robot_info.joint_info)
	P=np.array(robot.robot_info.chains[0].P.tolist())
	H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
	robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


	q=state_w.InValue.joint_position
	pose=state_w.InValue.kin_chain_tcp
	print(list(pose['position'][0]),q2R(list(pose['orientation'][0])))
	print(fwd(q))

if __name__ == "__main__":
	main()

