from Robot_Util import Robot
import numpy as np


## Manipulator Configuration
#robot = Robot([10, 10, 10], [np.pi/6, np.pi/6, -np.pi/4]) # Generic 3-link manipulator
robot = Robot([10, 10], [0, 0]) # Generic 2-link manipulator

def main():
    robot.visualize()

main()