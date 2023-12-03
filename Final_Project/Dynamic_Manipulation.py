from Robot_Util import Robot
import numpy as np


## Manipulator Configuration - link lengths, joint angles, link masses
link_lengths = [1,1,1]
initial_angles = [0,0,0]
initital_velocities = [0,0,0]
link_masses = [1,1,1]


## Create manipulator (uncomment one pair)
robot = Robot(link_lengths, initial_angles, initital_velocities, link_masses) # Custom manipulator using parameters above
torque_vector = [0,0,0]
##
# robot = Robot([1], [0], [0], [1]) # Generic 1-link gravity compensation
# torque_vector = [9,81]
##
# robot = Robot([1,1], [0,0], [0,0], [1,1]) # Generic 2-link gravity compensation
# torque_vector = [3*9.81, 9.81]
##
# robot = Robot([1,1,1], [0,0,0], [0,0,0], [1,1,1]) # Generic 3-link gravity compensation
# torque_vector = [6*9.81, 3*9.81, 9.81]
##


## Simulation Parameters
timestep = 50 # milliseconds
num_iterations = 100


def main():

    # Initialize array of robot states
    robot_poses = []

    # Simulate robot
    for x in range(num_iterations):
        joint_angles, joint_velocities = robot.fwd_dyn(torque_vector, timestep)
        pose = []
        for angle in joint_angles:
            pose.append(angle[0])
        robot_poses.append(pose)

    # Animate robot
    for x in range(num_iterations):
        robot.visualize(robot_poses[x], timestep)

main()