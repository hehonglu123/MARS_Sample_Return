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
import matplotlib.pyplot as plt

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


def move(vd, ER):
	global vel_ctrl, state_w, stop
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
		###Don't put bound here, will affect cartesian motion outcome
		qdot=solve_qp(H, f)
		###For safty, make sure robot not moving too fast
		if np.max(np.abs(qdot))>0.5:
			qdot=np.zeros(6)
			stop=True
			print('too fast')
		vel_ctrl.set_velocity_command(qdot)

	except:
		traceback.print_exc()
	return


def comp_control(F_x_des=1.):
	global robot_def, vel_ctrl, netft, state_w, force_reading, joint_position_reading

	
	###param to adjust center of compliance dynamically
	z_init=0.19499046
	d_EE_TCP=0.15
	d_EE_FT=0.033
	d_FT_COC=0

	###FT sensor reading
	res, ft, status = netft.try_read_ft_streaming(.1)
	torque_y=-np.sin(np.pi/4)*ft[0]+np.cos(np.pi/4)*ft[1]
	force_z=-np.sin(np.pi/4)*ft[3]-np.cos(np.pi/4)*ft[4]
	force_x=ft[-1]



	###small force filtering to avoid floating around
	# if np.abs(force_x)<0.15:
	# 	force_x=0.
	# if np.abs(force_z)<0.15:
	# 	force_z=0.

	###get theta (angle between first column of R_eef and robot y-axis):
	q_cur=state_w.InValue.joint_position
	pose=fwd(q_cur)
	R_cur=pose.R
	unit1=R_cur[:,0]/np.linalg.norm(R_cur[:,0])
	theta=np.arccos(np.dot(unit1,np.array([0,1,0])))


	###data recording
	force_reading.append([time.time(),force_x,force_z,torque_y])
	joint_position_reading.append(q_cur)

	###get FT_COC as moving down
	z_cur=pose.p[-1]
	d_EE_COC=z_cur-(z_init-d_EE_TCP)



	d_FT_COC=d_EE_COC-d_EE_FT
	print('d_FT_COC: ',d_FT_COC)

	###dynamic damping coefficient
	damping_coeff=max(np.linalg.norm(np.array([force_z,force_x,torque_y]))/2.,1)
	###pass to controller
	v_y_EE, v_z_EE, omega_EE = gen_damper_control(theta, force_z, force_x, torque_y, b_x=damping_coeff*2000., b_z=damping_coeff*1000., b_omega=damping_coeff*10., d_FT_COC=d_FT_COC, d_EE_COC=d_EE_COC, F_x_des=F_x_des)
	print('v_y: ', v_y_EE)
	print('v_z: ', v_z_EE)
	print('theta: ', theta)
	print('force_x: ',force_x)
	print('      ')

	###QP controller, cartesian linear motion and orientation
	vd=np.array([0,v_y_EE,v_z_EE])
	vd=np.clip(vd, -0.02, 0.02)
	ER=Rx(omega_EE)
	move(vd,ER)
	
def no_comp():
	global robot_def, vel_ctrl, netft, state_w, force_reading, joint_position_reading, stop

	###no compliance as comparison, issuing QP downward only
	res, ft, status = netft.try_read_ft_streaming(.1)
	torque_y=-np.sin(np.pi/4)*ft[0]+np.cos(np.pi/4)*ft[1]
	force_z=-np.sin(np.pi/4)*ft[3]-np.cos(np.pi/4)*ft[4]
	force_x=ft[-1]
	
	q_cur=state_w.InValue.joint_position
	###data recording
	force_reading.append([time.time(),force_x,force_z,torque_y])
	joint_position_reading.append(q_cur)

	vd=np.array([0,0,-0.01])
	move(vd,np.eye(3))

	if np.abs(force_x)>200:
		stop=True



def main():
	global robot_def, vel_ctrl, netft, state_w, force_reading, stop, joint_position_reading


	###jogging initial pose
	q_poffset=[ 1.57489506,  1.00884044, -0.06142472,  0.00267314,  0.61304244,  1.57120872]
	q_orioffset=[1.57526511e+00, 8.51933448e-01, 1.53590792e-01, 1.30709195e-03, 2.78377532e-01, 1.57228823e+00]
	q_perfect=[ 1.57480585,  0.99928189, -0.07043686,  0.00262175,  0.63853202,  1.57113588]

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


	###jogging to intial pose
	jog_joint(q_orioffset,0.2)

	###get robot kinematics parameters
	num_joints=len(robot.robot_info.joint_info)
	P=np.array(robot.robot_info.chains[0].P.tolist())
	H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
	robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))

	q=vel_ctrl.joint_position()
	pose=fwd(q)
	R_cur=pose.R
	print(R_cur)

	###FT sensor initialization
	netft=rpi_ati_net_ft.NET_FT('192.168.50.65')
	netft.set_tare_from_ft()
	netft.start_streaming()

	###enable velocity emulation
	vel_ctrl.enable_velocity_mode()
	###FT sensor clear gravity bias
	netft.set_tare_from_ft()
	Fd=100.	#desired force set to 100N
	joint_position_reading=[]
	force_reading=[]
	stop=False
	now=time.time()

	###live plots initialization
	max_idx=100
	plt.ion()
	fig = plt.figure()
	ax = fig.add_subplot(111)


	line_fz, = ax.plot([1,2], [1,2], 'b-')
	line_fx, = ax.plot([1,2],[1,2], 'r-')
	line_ty, = ax.plot([1,2],[1,2], 'g-')

	last_time=time.time()
	while True:
		try:
			comp_control(Fd)
			# no_comp()

			###plotting only every 10ms
			if time.time()-last_time>0.01:
				force_reading_nd=np.array(force_reading)
				force_reading_nd[:,0]-=now
				line_fz.set_xdata(force_reading_nd[max(0,len(force_reading)-max_idx):,0])
				line_fz.set_ydata(force_reading_nd[max(0,len(force_reading)-max_idx):,1])
				line_fx.set_xdata(force_reading_nd[max(0,len(force_reading)-max_idx):,0])
				line_fx.set_ydata(force_reading_nd[max(0,len(force_reading)-max_idx):,2])
				line_ty.set_xdata(force_reading_nd[max(0,len(force_reading)-max_idx):,0])
				line_ty.set_ydata(force_reading_nd[max(0,len(force_reading)-max_idx):,3])


				ax.relim()
				ax.autoscale_view()
				fig.canvas.draw()
				fig.canvas.flush_events()

				last_time=time.time()

			if stop:
				break

			time.sleep(0.001)
		except:
			vel_ctrl.disable_velocity_mode()
			traceback.print_exc()
			break

	
	plt.xlabel('time (s)')
	plt.ylabel('force (N)')
	plt.legend(['force z','force_y'])
	plt.show()

	print(len(force_reading),len(joint_position_reading))
	print(force_reading[-1])
	final_data=np.hstack((np.array(force_reading),np.array(joint_position_reading)))
	np.savetxt("final_data"+str(int(time.time()))+".csv", final_data, delimiter=",")

if __name__ == "__main__":
	main()

