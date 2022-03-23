#!/usr/bin/env python3
from RobotRaconteur.Client import *
import time
import numpy as np
import sys

robot = RRN.ConnectService('rr+tcp://pi-tormach:11111?service=tormach_robot')        #Sawyer


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode



robot.jog_freespace(np.zeros(6), np.ones((6,)), False)
