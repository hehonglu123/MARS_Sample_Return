from RobotRaconteur.Client import *
import numpy as np
import time,traceback, sys, yaml
from importlib import import_module
from general_robotics_toolbox import *
sys.path.append('toolbox')
from vel_emulate_sub import EmulatedVelocityControl
from qpsolvers import solve_qp
from robots_def import *
from gen_damper_control import *
import matplotlib.pyplot as plt


def wrench_wire_cb(w,value,time):
	global torque, force
	torque=np.array(value['torque']['x'],value['torque']['y'],value['torque']['z'])
	force=np.array(value['force']['x'],value['force']['y'],value['force']['z'])

def move(vd, ER):
	global vel_ctrl, state_w, stop, robot_toolbox
	try:
		w=1.
		Kq=.01*np.eye(6)    #small value to make sure positive definite
		KR=np.eye(3)        #gains for position and orientation error

		q_cur=vel_ctrl.joint_position()
		J=robot_toolbox.jacobian(q_cur)        #calculate current Jacobian
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

def main():
	global vel_ctrl, torque, force, state_w, stop, robot_toolbox

	with open('config/abb1200.yml') as robot_file:
		with open('config/tool_pose.yaml') as tool_file:
			robot_toolbox=yml2robdef(robot_file,tool_file)

	################################Connect to FT Sensor###################################
	url='rr+tcp://localhost:59823?service=ati_sensor'
    if (len(sys.argv)>=2):
        url=sys.argv[1]

    #Connect to the service
    cli = RRN.ConnectService(url)

    #Connect a wire connection
    wrench_wire = cli.wrench_sensor_value.Connect()

    #Add callback for when the wire value change
    wrench_wire.WireValueChanged += wrench_wire_cb

	##########################Connect Robot Service###########################################

	robot_sub=RRN.SubscribeService('rr+tcp://pi-tormach:11111?service=tormach_robot')
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

	###enable velocity emulation
	vel_ctrl.enable_velocity_mode()

	while True:
		try:
			#convert tool frame to base frame
			q_cur=vel_ctrl.joint_position()
			R=robot_toolbox.fwd(q_cur).R
			torque_bf=np.dot(R,torque)
			force_bf=np.dot(R,force)
			#command motion based on reading
			K_force=0.001
			K_torque=0.1
			move(K_force*np.array([force_bf[0],0,force_bf[-1]]),Rz(K_torque*torque_bf[1]))
		except:
			traceback.print_exc()
			break

	vel_ctrl.set_velocity_command(np.zeros(6))
	vel_ctrl.disable_velocity_mode()

if __name__ == "__main__":
	main()