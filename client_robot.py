#Simple example Robot Raconteur Robot Print joint client
from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module
from general_robotics_toolbox import *
sys.path.append('toolbox')
from vel_emulate_sub import EmulatedVelocityControl
from qpsolvers import solve_qp
import rpi_ati_net_ft
from gen_damper_control import *

def normalize_dq(q):
	q[:-1]=q[:-1]/(np.linalg.norm(q[:-1])) 
	return q  

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

def Rx(theta):
	return np.array(([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]]))

def Rz(theta):
	return np.array(([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]))


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


# def moveL(pd,Rd):
# 	global vel_ctrl, state_w
# 	q_cur=state_w.InValue.joint_position
# 	pose=state_w.InValue.kin_chain_tcp
# 	R_cur = q2R(list(pose['orientation'][0]))
# 	p_cur = list(pose['position'][0])

# 	R_temp=np.dot(R_cur.T,Rd)
# 	k,theta=R2rot(R_temp)

# 	p_interp=[]
# 	R_interp=[]
# 	for i in range(10000):
# 		###interpolate orientation first
# 		p_interp.append(p_cur+(pd-p_cur)*i/10000.)
# 		###interpolate orientation second
# 		angle=theta*i/10000.
# 		R=rot(k,angle)
# 		R_interp.append(np.dot(R_cur,R))

# 	###issuing position command
# 	command_seqno = 1
# 	for i in range(len(p_interp)):
# 		###TODO:
# 		q_all=inv(p_interp[i],R_interp[i])
# 		#find closest joint config
# 		if i==0:
# 			temp_q=q_all-q_cur
# 			order=np.argsort(np.linalg.norm(temp_q,axis=1))
# 			q_next=q_all[order[0]]
# 		else:
# 			temp_q=q_all-q_next
# 			order=np.argsort(np.linalg.norm(temp_q,axis=1))
# 			q_next=q_all[order[0]]


# 		joint_cmd = RobotJointCommand()
# 		joint_cmd.seqno = command_seqno
# 		joint_cmd.state_seqno = state_w.InValue.seqno
# 		cmd = q_next
# 		joint_cmd.command = cmd
# 		cmd_w.OutValue = joint_cmd
# 		command_seqno += 1



def move(vd, ER):
	global vel_ctrl, state_w
	try:
		w=1.
		Kq=.01*np.eye(6)    #small value to make sure positive definite
		KR=np.eye(3)        #gains for position and orientation error

		q_cur=vel_ctrl.joint_position()
		J=jacobian(q_cur)        #calculate current Jacobian
		Jp=J[3:,:]
		JR=J[:3,:] 
		H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)

		H=(H+np.transpose(H))/2


		k,theta = R2rot(ER)
		k=np.array(k)
		s=np.sin(theta/2)*k         #eR2
		wd=-np.dot(KR,s)  
		f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)

		qdot=solve_qp(H, f)
		if np.max(np.abs(qdot))>0.4:
			qdot=np.zeros(6)
			print('too fast')
		vel_ctrl.set_velocity_command(qdot)

	except:
		traceback.print_exc()
	return


def main():
	global robot_def, vel_ctrl, joint_bias
	####################Start Service and robot setup

	robot_sub=RRN.SubscribeService('rr+tcp://pi_fuse:58651?service=robot')
	robot=robot_sub.GetDefaultClientWait(1)
	state_w = robot_sub.SubscribeWire("robot_state")
	cmd_w = robot_sub.SubscribeWire("position_command")
	RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)
	
	robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
	state_flags_enum = robot_const['RobotStateFlags']
	halt_mode = robot_const["RobotCommandMode"]["halt"]
	position_mode = robot_const["RobotCommandMode"]["position_command"]
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = position_mode


	print(robot.robot_info.device_info.device.name+" Connected")

	num_joints=len(robot.robot_info.joint_info)
	P=np.array(robot.robot_info.chains[0].P.tolist())
	H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
	robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))

	Rd=np.array([[0,0,1],[0,1,0],[-1,0,0]])

	q=vel_ctrl.joint_position()
	pose=fwd(q)
	R_cur=pose.R
	print(R_cur)

	netft=rpi_ati_net_ft.NET_FT('192.168.50.65')
	netft.set_tare_from_ft()
	netft.start_streaming()
	now=time.time()


	vel_ctrl.enable_velocity_mode()
	while True:
		try:
			res, ft, status = netft.try_read_ft_streaming(.1)
			torque_y=-np.sin(np.pi/4)*ft[0]+np.cos(np.pi/4)*ft[1]
			force_z=-np.sin(np.pi/4)*ft[3]-np.cos(np.pi/4)*ft[4]
			force_x=ft[-1]

			if np.abs(force_x)<0.15:
				force_x=0.
			if np.abs(force_z)<0.15:
				force_z=0.

			v_y_EE, v_z_EE, omega_EE = gen_damper_control(np.pi/2, force_z, force_x, 0., b_x=50., b_z=50., b_omega=50., d_FT_COC=0.233-0.033, d_EE_COC=0.233, F_x_des=0.)
			print('v_y: ', v_y_EE)
			print('v_z: ', v_z_EE)
			print('omega: ',omega_EE)
			print('      ')

			vd=np.array([0,v_y_EE,v_z_EE])
			vd=np.clip(vd, -0.2, 0.2)
			ER=Rx(-omega_EE)
			move(vd,np.eye(3))
			time.sleep(0.001)
		except:
			vel_ctrl.disable_velocity_mode()
			break

	# pose_p=np.array([-0.00772909, 0.66492853, 0.5176524])
	# pose_R=np.array([[np.pi/2,0,0,0,0,0],R=np.array([[ 0,0,1],
	# 		 [0,1,0],
	# 		 [-1,0,0]])
	# pose_q=[ 1.58376339,  0.5813696,  -0.09457617, -0.01236589,  1.0756525,   1.59067882]
	# jog_joint(pose_q)


	# now=time.time()
	# vel_ctrl.enable_velocity_mode()
	# while time.time()- now<5:
	# 	vd=[0,-0.01,0]
	# 	move(vd,np.eye(3))
	# vel_ctrl.disable_velocity_mode()


if __name__ == "__main__":
	main()

