#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module
from general_robotics_toolbox import *
sys.path.append('toolbox')
from vel_emulate_sub import EmulatedVelocityControl

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

def move(vd, Rd):
	global vel_ctrl
	try:
		w=1.
		Kq=.01*np.eye(n)    #small value to make sure positive definite
		KR=np.eye(3)        #gains for position and orientation error

		q_cur=vel_ctrl.joint_position()
		J=jacobian(q_cur)        #calculate current Jacobian
		Jp=J[3:,:]
		JR=J[:3,:] 
		H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)

		H=(H+np.transpose(H))/2

		robot_pose=fwdkin(robot_def,q_cur.reshape((n,1)))
		R_cur = robot_pose.R
		ER=np.dot(R_cur,np.transpose(R_ee.R_ee(0)))
		k,theta = R2rot(ER)
		k=np.array(k)
		s=np.sin(theta/2)*k         #eR2
		wd=-np.dot(KR,s)  
		f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)
		qdot=0.5*normalize_dq(solve_qp(H, f))
		vel_ctrl.set_velocity_command(qdot)

		jobid = top.after(10, lambda: move(n, robot_def,vel_ctrl,vd))
	except:
		traceback.print_exc()
	return


def main():
	global robot_def, vel_ctrl
	####################Start Service and robot setup

	robot_sub=RRN.SubscribeService('rr+tcp://pathpilot:11111?service=tormach_robot')
	robot=robot_sub.GetDefaultClientWait(1)
	state_w = robot_sub.SubscribeWire("robot_state")
	cmd_w = robot_sub.SubscribeWire("position_command")
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)

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

