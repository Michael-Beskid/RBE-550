from Robot_Util import Robot
import numpy as np
import time


## Manipulator Configuration - link lengths, joint angles, link masses
link_lengths = [1,1]
initial_angles = [0,0]
link_masses = [1,1]

## Create manipulator
robot = Robot(link_lengths, initial_angles, link_masses) # Generic 2-link manipulator

## Simulation Parameters
torque_vector = [0,0]
start_delay = 1000 # milliseconds
timestep = 50 # milliseconds
num_iterations = 50

def main():
    # Show robot
    robot.visualize(1000)

    for x in range(num_iterations):
        # Simulate robot
        joint_angles, joint_velocities = robot.fwd_dyn(torque_vector, timestep)

        # Update robot display
        robot.visualize(timestep)

main()