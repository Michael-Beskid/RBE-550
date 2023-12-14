from Robot_Util import Robot
from RRT import RRT, Node
from Map2D import Map
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
num_iterations = 200


def main():

    # Initialize array of robot states
    robot_poses = []

    # Load map
    map_2d = Map("maps/petermap.csv")
    obstacles, obstacle_edges = map_2d.get_obstacles()

    ### UNCOMMENT TO CHECK STATE VALIDATOR ###
    # robot1 = Robot([1,1,1], [0,0,0], [0,0,0], [1,1,1]) # Should pass
    # robot2 = Robot([1,1,1], [np.pi/4,0,0], [0,0,0], [1,1,1]) # Should fail due to obstacle collision
    # robot3 = Robot([1,1,1], [0,0,0], [0,0,-20], [1,1,1]) # Should fail due to joint velocity limit
    # print("Robot 1: Is Valid State?")
    # print ("Yes" if robot1.isValidState([0,0,0], obstacles, obstacle_edges) else "No")
    # robot1.visualize([0,0,0],obstacles,2000)ss
    # print("Robot 2: Is Valid State?")
    # print ("Yes" if robot2.isValidState([0,0,0], obstacles, obstacle_edges) else "No")
    # robot2.visualize([np.pi/4,0,0],obstacles,2000)
    # print("Robot 3: Is Valid State?")
    # print ("Yes" if robot3.isValidState([0,0,0], obstacles, obstacle_edges) else "No")
    # robot3.visualize([0,0,0],obstacles,2000)

    ### UNCOMMENT TO SIMULATE THE ROBOT FOR SOME TIME AND THEN ANIMATE IT ###
    # # Simulate robot
    # for x in range(num_iterations):
    #     joint_angles, joint_velocities = robot.fwd_dyn(torque_vector, timestep)
    #     pose = []
    #     for angle in joint_angles:
    #         pose.append(angle[0])
    #     robot_poses.append(pose)
    # # Animate robot
    # for x in range(num_iterations):
    #     robot.visualize(robot_poses[x], obstacles, timestep)

    # Initialize planner
    rrt_planner = RRT(robot, map_2d, np.array([np.pi/2,0,0,0,0,0]), np.array([(np.pi/2)-0.5,0,0,0,0,0]))

    # Compute path
    path = rrt_planner.run(n_samples=1000, n_iterations=10)

    # Animate path
    for node in path:
        print(f"===debug===\nstart pose: {node.plotting_poses[0]}\nendpose:{node.plotting_poses[-1]}")
        for pose in node.plotting_poses:
            
            robot.visualize(pose, obstacles, timestep)
        # print(f"node state: {node.state}")

    


main()
