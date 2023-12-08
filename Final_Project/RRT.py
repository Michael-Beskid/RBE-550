from Robot_Util import Robot
import numpy as np

# RRT-based kinodynamic planner 

class Node:
    ''' RRT Node object. '''
    def __init__(self, t1, t2, t3, w1, w2, w3):
        self.t1 = t1; self.t2 = t2; self.t3 = t3    # angle for joints 1, 2, 3
        self.w1 = w1; self.w2 = w2; self.w3 = w3    # angular velocity for joints 1, 2, 3
        self.parent = None

class RRT:
    ''' RRT kinodynamic planner object used to compute a valid trajectory around obstacles. '''
    def __init__(self, robot, goal, start=None):
        self.robot = robot      # robot class as defined in Robot_Util
        self.tree = []        # list of nodes as they're added 
        self.start = Node(*robot.joint_angles[:3], *robot.joint_velocities[:3]) if None else start
        self.goal = Node(*goal)  # desired end position of robot

    def run(self, n_samples = 100, n_iterations = 10):
        ''' 
        Execute main search function 
            Arguments:
                n_samples : int, number of samples to execute before failing
                n_iterations : int, number of iterations to try during a qnew search 
            Returns:
                found : bool, true if found, false if not
        '''

        print(f"You have reached the main function, lmao")
        print(f"your robot: {Robot}\nyour start: {self.start}\nyour end: {self.goal}\n")
        print(f"Executing {n_samples} samples. Qnew iterations: {n_iterations}")

        # pick q_rand

        # find closest node to bridge FROM

        # find and add q_new to tree (based on iterative search)

        # check if we've made it

