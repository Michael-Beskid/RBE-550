from Robot_Util import Robot
import numpy as np
from scipy.spatial import KDTree
import copy

# RRT-based kinodynamic planner 

class Node:
    ''' RRT Node object. '''
    def __init__(self, state, parent=None, torques_req = [0,0,0]):
        self.state = state
        self.t1, self.t2, self.t3, self.w1, self.w2, self.w3 = state

        self.parent = parent
        self.torques_req = torques_req
    
    def __iter__(self): # iterator returns state space parameters when you type *[node name here]
        yield self.t1; yield self.t2; yield self.t3
        yield self.w1; yield self.w2; yield self.w3

    def debug(self):
        print(f"So you're debugging {self} huh... \nstate: {self.state}, \t parent: {self.parent}")

class RRT:
    ''' RRT kinodynamic planner object used to compute a valid trajectory around obstacles. '''
    def __init__(self, robot, map, goal, start=None):
        self.robot = robot          # robot class as defined in Robot_Util
        self.map = map              # Map2D object for collision checking
        self.tree = []              # list of nodes as they're added 
        self.start = Node([*robot.joint_angles[:3], *robot.joint_velocities[:3]]) if start.any()==None else Node([*start])  # start pose of robot if given. o/w whatever state the robot is in now.  #TODO: Fix this because it's creating numpy arrays within arrays ;(
        self.goal = Node([*goal])     # desired end pose of robot
        self.found = False          # found flag

        # == initialize kd tree here ==

        print("== init RRT =="); print("init KD tree"); print(f"start.state: {self.start.state}\t goal.state {self.goal.state}")
        self.kdtree = KDTree(np.vstack((self.start.state,self.start.state))) 
        print(self.kdtree)
        self.tree.append(self.start)
        print(f"also, I added start and goal to our self.tree: {self.tree}")

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

        # print(self.start.debug()) # use this to debug a node

        # for number of samples:
            # pick q_rand
            # find closest node to q_rand
            # extend tree towards q_rand (iterative simulation)
            # check to see if we've made it

        # print(f"The distance between start and goal is {self.distance(self.start, self.goal)} (... units?) ") # test distance

        # self.init(self.start, self.goal)

        for _ in range(n_samples):

            qrand = self.sample()
            print(f"qrand: {qrand}")

            qnew = self.extend(qrand, n_iterations = n_iterations)

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
            return self.goal.state
        
        # c space limits, TODO: pull these from robot?

        
        return np.array([np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi),
               np.random.uniform(0, 10), np.random.uniform(0, 10), np.random.uniform(0, 10)])
        

    def extend(self, qrand, n_iterations):
        '''
        Extend the tree in the direction of qrand, our randomly-sampled node.
            Arguments:
                qrand : Node, randomly sampled node
            Returns:
                qnew : Node, node in state space that moves towards qrand
        '''

        # find closest existing node to bridge from
        print("finding closest existing node to qrand...")
        distance, index = self.kdtree.query(qrand)
        bridge_from_state = self.kdtree.data[index]
        print(f"nearest to qrand: {bridge_from_state} at distance: {distance}")
        # find this state's corresponding node
        for i, node in enumerate(self.tree):
            if (bridge_from_state == node.state).all():
                parent = node

        # bridge from that node
        print("simulating to find possible branch to qnew ...")

        valid_sims = {} # initialze simulation dictionary
        for i in range(n_iterations):

            # generate random torque values for each joint
            random_torques = np.random.uniform(0,10,3) # critical value here, min and max torque
            robot_copy = copy.deepcopy(self.robot)  # create a complete copy of our robot to simultate fresh

            is_valid, robot_poses, end_state = self.simulate(robot_copy, random_torques, 10)

            if is_valid:
                dis = self.distance(qrand, np.array(end_state))
                valid_sims[i] = {"end_state": end_state,
                                 "robot_poses" : robot_poses,
                                 "torques": random_torques,
                                 "distance": dis}
            
        # get lowest distance from valid simulations, thats your new node
        min_distance_key = min(valid_sims, key=lambda x: valid_sims[x]['distance'])
        print(f"THIS SHOULD BE A STATE: {valid_sims[min_distance_key]['end_state']}")
        self.add_node(parent, valid_sims[min_distance_key]['end state'], valid_sims[min_distance_key]['torques'])

        pass

    def check(self):
        '''
        Checks to see if our planner has made it close to the goal.
            Returns:

        '''
        pass

    # === Helper Functions ===


    def distance_nodes(self, P, Q):
        ''' 
        Distance Metric from points P and Q in our state space
            Arguments:
                P, Q : Nodes of our tree
            Returns
                Euclidean distance between the two nodes (states)
        ''' 
        return np.linalg.norm(np.array([*Q]) - np.array([*P])) #TODO: add manhattan distance or some other more sophisticated metric
    
    def distance(self, P, Q):
        return np.linalg.norm(P - Q)
    
    def simulate(self, robot, torque, steps):
        '''
        Simulate robot forward in time.
            Arguments:
                robot : robot object, as defined in Robot_Util.py
                torque : 1x3 list of joint torques (nm) 
                steps : int, number of iterations for the numerical solver
                duration : float, length of time simulation runs (seconds)
            Returns:
                is_valid, bool, is simulation in collision or not
                robot_poses : nx3 list of joint positions 
                end_state : 1x6 list of the ending joint positions and velocities
        '''
        robot_poses = []
        robot_states = []
        timestep = 50 # duration in ms
        
        print(f"\tsimulating {(steps * timestep)/1000} s")

        obstacles, obstacle_edges = self.map.get_obstacles()
        for x in range(steps):
            joint_angles, joint_velocities = robot.fwd_dyn(torque, timestep)
            
            robot_states.append(np.concatenate(joint_angles.reshape((1,3)), joint_velocities.reshape((1,3))))
            print(robot_states)
            
 
            is_valid = self.robot.isValidState(joint_angles, obstacles, obstacle_edges)
            if not is_valid:
                print("simulation collision")
                break

        # debug visualization
        # for x in range(steps):
        #     robot.visualize(robot_poses[x], obstacles, timestep)
        
        end_state = robot_states[-1]

        return is_valid, robot_poses, end_state
    
    def add_node(self, parent, qnew_state, qnew_torques):
        '''
        Add a valid state to the tree.
            Arguments:
                qnew_state : 1x6 list, state of new node
                qnew_torques : 1x3 list, torques applied to arrive at qnew_state
        '''
        
        # apparently we can't add to kd tree so I guess whenever we query we'll rebuild
        # add node
        self.tree.append(Node(qnew_state, parent, qnew_torques))
