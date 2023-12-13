import numpy as np
from Robot_Util import Robot

## Manipulator Configuration - link lengths, joint angles, link masses
link_lengths = [1]
initial_angles = [0]
initital_velocities = [0]
link_masses = [1]


## Create manipulator (uncomment one pair)
robot = Robot(link_lengths, initial_angles, initital_velocities, link_masses) # Custom manipulator using parameters abo

holding_torque = robot.calc_holding_torque()

print(holding_torque)