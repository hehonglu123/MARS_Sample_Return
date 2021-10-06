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

def jog_joint(q,max_v,vd=[]):
	global vel_ctrl
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()

	diff=q-vel_ctrl.joint_position()
	time_temp=np.linalg.norm(diff)/max_v

	qdot_temp=np.clip((1.15*diff)/time_temp,-max_v,max_v)
	while np.linalg.norm(q-vel_ctrl.joint_position())>0.01:
		
		diff=q-vel_ctrl.joint_position()
		qdot=np.where(np.abs(diff) > 0.05, qdot_temp, diff)

		vel_ctrl.set_velocity_command(qdot)
	vel_ctrl.set_velocity_command(np.zeros((6,)))
	vel_ctrl.disable_velocity_mode() 


def moveL(pd,Rd):
	global vel_ctrl, state_w
	q_cur=state_w.InValue.joint_position
	pose=state_w.InValue.kin_chain_tcp
	R_cur = q2R(list(pose['orientation'][0]))
	p_cur = list(pose['position'][0])

	R_temp=np.dot(R_cur.T,Rd)
	k,theta=R2rot(R_temp)

	p_interp=[]
	R_interp=[]
	for i in range(10000):
		###interpolate orientation first
		p_interp.append(p_cur+(pd-p_cur)*i/10000.)
		###interpolate orientation second
		angle=theta*i/10000.
		R=rot(k,angle)
		R_interp.append(np.dot(R_cur,R))

	###issuing position command
	command_seqno = 1
	for i in range(len(p_interp)):
		###TODO:
		q_all=inv(p_interp[i],R_interp[i])
		#find closest joint config
		if i==0:
			temp_q=q_all-q_cur
			order=np.argsort(np.linalg.norm(temp_q,axis=1))
			q_next=q_all[order[0]]
		else:
			try:
				temp_q=q_all-q_next
				order=np.argsort(np.linalg.norm(temp_q,axis=1))
				q_next=q_all[order[0]]


		joint_cmd = RobotJointCommand()
	    joint_cmd.seqno = command_seqno
	    joint_cmd.state_seqno = state_w.InValue.seqno
	    cmd = q_next
	    joint_cmd.command = cmd
	    cmd_w.OutValue = joint_cmd
	    command_seqno += 1



def move(vd, Rd):
	global vel_ctrl, state_w
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

		R_cur = fwdkin(q_cur).R
		ER=np.dot(R_cur,np.transpose(Rd))
		k,theta = R2rot(ER)
		k=np.array(k)
		s=np.sin(theta/2)*k         #eR2
		wd=-np.dot(KR,s)  
		f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)
		# lb=-0.2*np.ones(6)
		# ub=0.2*np.ones(6)
		qdot=0.5*normalize_dq(solve_qp(H, f))
		vel_ctrl.set_velocity_command(qdot)

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
	RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)
	###enable velocity mode
	# vel_ctrl.enable_velocity_mode()


	print(robot.robot_info.device_info.device.name+" Connected")

	num_joints=len(robot.robot_info.joint_info)
	P=np.array(robot.robot_info.chains[0].P.tolist())
	H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
	robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))


	q=state_w.InValue.joint_position
	pose=state_w.InValue.kin_chain_tcp
	print(list(pose['position'][0]),q2R(list(pose['orientation'][0])))
	print(fwd(q))

	jog_joint(np.ones(6),0.1)

if __name__ == "__main__":
	main()

