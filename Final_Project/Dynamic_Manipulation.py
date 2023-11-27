from Robot_Util import Robot
import numpy as np
import time


## Manipulator Configuration - link lengths, joint angles, link masses
#robot = Robot([10, 10, 10], [np.pi/6, np.pi/6, -np.pi/4], [1, 1, 1]) # Generic 3-link manipulator
robot = Robot([0.5, 0.5], [0, 0], [1, 1]) # Generic 2-link manipulator


## Simulation Parameters
torque_vector = [0, 0]
start_delay = 1000 # milliseconds
timestep = 50 # milliseconds

def main():
    # Show robot
    robot.visualize(1000)

    for x in range(30):
        # Simulate robot
        joint_angles, joint_velocities = robot.fwd_dyn(torque_vector, timestep)

        # Update robot display
        robot.visualize(timestep)

main()