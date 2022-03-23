import numpy as np
from pandas import *
import sys, traceback
from general_robotics_toolbox import *
import matplotlib.pyplot as plt
sys.path.append('toolbox')
from abb_def import *


def main():

	filename='final_data(nocomp-poffset)'

	col_names=['time', 'force_z', 'force_x','torque_y','q1','q2','q3','q4','q5','q6'] 
	data = read_csv(filename+".csv", names=col_names)
	timestamp=np.array(data['time'].tolist())
	# timestamp-=timestamp[0]
	q1=data['q1'].tolist()
	q2=data['q2'].tolist()
	q3=data['q3'].tolist()
	q4=data['q4'].tolist()
	q5=data['q5'].tolist()
	q6=data['q6'].tolist()


	q=np.vstack((q1, q2, q3, q4, q5, q6)).T


	theta=[]
	pos=[]
	

	for i in range(len(timestamp)):
		pose=fwd(q[i])
		pos.append(pose.p)
		R=pose.R
		unit1=R[:,0]/np.linalg.norm(R[:,0])
		theta.append(np.arccos(np.dot(unit1,np.array([0,1,0]))))


	pos=np.array(pos)
	theta=np.degrees(np.array(theta)).reshape((-1,1))


	np.savetxt(filename+'-fwd'+".csv", np.hstack((timestamp.reshape((-1,1)),np.hstack((pos,theta)))), delimiter=",")

	# plt.plot(timestamp,theta)
	# # plt.plot(timestamp,pos[:,1])
	# # plt.plot(timestamp,pos[:,2])

	# plt.xlabel('time (s)')
	# plt.ylabel('position (m)')

	# plt.legend(['x','y','z'])
	# plt.show()

if __name__ == "__main__":
	main()