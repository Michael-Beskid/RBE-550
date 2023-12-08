from Robot_Util import Robot
import numpy as np

# RRT-based kinodynamic planner 

class Node:
    ''' RRT Node object. '''
    def __init__(self, t1, t2, t3, w1, w2, w3):
        self.t1 = t1; self.t2 = t2; self.t3 = t3    # angle for joints 1, 2, 3
        self.w1 = w1; self.w2 = w2; self.w3 = w3    # angular velocity for joints 1, 2, 3
        self.parent = None
    
    def __iter__(self): # iterator returns state space parameters when you type *[node name here]
        yield self.t1; yield self.t2; yield self.t3
        yield self.w1; yield self.w2; yield self.w3


class RRT:
    ''' RRT kinodynamic planner object used to compute a valid trajectory around obstacles. '''
    def __init__(self, robot, goal, start=None):
        self.robot = robot          # robot class as defined in Robot_Util
        self.tree = []              # list of nodes as they're added 
        self.start = Node(*robot.joint_angles[:3], *robot.joint_velocities[:3]) if start==None else start  # start pose of robot if given. o/w whatever state the robot is in now. 
        self.goal = Node(*goal)     # desired end pose of robot
        self.found = False          # found flag

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

        # for number of samples:
            # pick q_rand
            # find closest node to q_rand
            # extend tree towards q_rand (iterative simulation)
            # check to see if we've made it
        print(f"The distance between start and goal is {self.distance(self.start, self.goal)} (... units?) ")

        for _ in range(n_samples):
            
            qrand = self.sample()

            qnew = self.extend(qrand)

            self.check()


    def sample(self, goal_bias = 0.1):
        ''' 
        Randomly sample the state space.
            Arguments:
                goal_bias : float, chance of selecting the goal state as your random sample
            Returns:
                qrand : Node, random sample of state space
        '''

        # goal bias
        if np.random.rand() <= goal_bias:
            return self.goal
        
        # c space limits, TODO: pull these from robot?
        
        return Node(np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi),
                    np.random.uniform(0, 10), np.random.uniform(0, 10), np.random.uniform(0, 10))
        

    def extend(self, qrand):
        '''
        Extend the tree in the direction of qrand, our randomly-sampled node.
            Arguments:
                qrand : Node, randomly sampled node
            Returns:
                qnew : Node, node in state space that moves towards qrand
        '''

        # find closest existing node to bridge from

        # bridge from that node

        pass

    def check(self):
        '''
        Checks to see if our planner has made it close to the goal.
            Returns:

        '''
        pass

    # === Helper Functions ===


    def distance(self, P, Q):
        ''' Distance Metric from points P and Q in our state space ''' #TODO: add manhattan distance or some other more sophisticated metric
        return np.linalg.norm(np.array([*Q]) - np.array([*P]))