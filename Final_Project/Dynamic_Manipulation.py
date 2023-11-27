from Robot_Util import Robot
import numpy as np
import time


## Manipulator Configuration
#robot = Robot([10, 10, 10], [np.pi/6, np.pi/6, -np.pi/4], [1, 1, 1]) # Generic 3-link manipulator
robot = Robot([0.5, 0.5], [0, 0], [1, 1]) # Generic 2-link manipulator

def main():
    # Show robot
    robot.visualize()

    for x in range(15):
        # Simulate robot
        joint_angles, joint_velocities = robot.fwd_dyn([0, 0], 100)
        print(joint_angles)
        print(joint_velocities)

        # Update robot display
        robot.visualize()

main()