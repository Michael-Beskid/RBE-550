from Robot_Util import Robot
import numpy as np
from scipy.spatial import KDTree
import copy

# RRT-based kinodynamic planner 

class Node:

    ''' 
    RRT Node object. 
        Attributes:
            state : 1 x 6 np.array of floats representing theta 1, 2, 3, omega 1, 2, 3
            t1, t2, t3, w1, w2, w3 : values for each angle, angular velocity in state 
            parent : Node associated with this Node's parent
            torques_req : 1 x 3 float of torques needed to move to this Node
            plotting_poses : for visualizations: n x 6 array of floats representing slices of our simulation (from the previous Node, to this Node) to be able to plot when we're done
    '''

    def __init__(self, state, plotting_poses = None, parent=None, torques_req = [0,0,0]):
        self.state = state
        self.t1, self.t2, self.t3, self.w1, self.w2, self.w3 = state
        self.parent = parent
        self.torques_req = torques_req
        self.plotting_poses = plotting_poses
    
    def __iter__(self): # iterator returns state space parameters when you type *[node name here]
        yield self.t1; yield self.t2; yield self.t3
        yield self.w1; yield self.w2; yield self.w3

    def debug(self):
        print(f"So you're debugging {self} huh... \nstate: {self.state}, \t parent: {self.parent}")


class RRT:
    
    ''' 
    RRT kinodynamic planner object used to compute a valid trajectory around obstacles. 
        Attributes:
            robot : robot class as specified in Robot_Util.py you want to plan with
            map : Map2D class as specified in Utils.py containing your obstacles
            tree : list of Nodes representing the RRT tree.
            start : Node representing the beginning of the RRT tree
            goal : Node representing the end position of the RRT tree. Note that we're just storing it as a node, it doesn't really have to be
            found : boolean representing if our planner has succeeded or failed
            path : list of Nodes in our found path
        Methods:
            run() : Execute main run function.
            sample() : Randomly sample the state space.
            extend() : Extend the tree in the direction of qrand, our randomly-sampled node.
            check() : Checks to see if our planner has made it close to the goal (success condition).
            distance() : Distance metric from two states in our state space.
            simulate() : Simulate robot forward in time.
            add_node() : Add a valid state as a Node into the tree.
            construct_path() : Recurse through RRT tree to construct a valid path.
    '''

    def __init__(self, robot, map, goal, start=None):
        self.robot = robot          # robot class as defined in Robot_Util
        self.map = map              # Map2D object for collision checking
        self.tree = []              # list of nodes as they're added
        # self.start = Node([*robot.joint_angles[:3], *robot.joint_velocities[:3]], [*robot.joint_angles[:3], *robot.joint_velocities[:3]]) if start.any()==None else Node([*start])  # start pose of robot if given. o/w whatever state the robot is in now.  #TODO: Fix this because it's creating numpy arrays within arrays ;(
        self.start = Node(start, # node state
                          [start, start]) # plotting poses
        self.goal = Node([*goal])   # desired end pose of robot
        self.found = False          # found flag
        self.path = []              # list of nodes in our found path

        # == initialize kd tree here ==

        print("== init RRT =="); print("init KD tree"); print(f"start.state: {self.start.state}\t goal.state {self.goal.state}")
        self.kdtree = KDTree(np.vstack((self.start.state,self.start.state))) 
        print(self.kdtree)
        self.tree.append(self.start)
        print(f"added start and goal to our self.tree: {self.tree}")


    def run(self, n_samples = 100, n_iterations = 10):
        
        ''' 
        Execute main search function 
            Arguments:
                n_samples : int, number of samples to execute before failing
                n_iterations : int, number of iterations to try during a qnew search 
            Returns:
                path : list of nodes in found path
        '''

        print(f"You have reached the main function")
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
            # print(f"qrand: {qrand}")

            self.extend(qrand, n_iterations = n_iterations)

            self.check(self.tree[-1]) # qnew is the last node of the tree

            if self.found:
                print(self.path)
                print("Path found!")
                return self.path 
            
            print('')
            
        print("\nMaximum iterations reached. No path found.\n")
        

    def sample(self, goal_bias = 0.2):

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
        
        return np.array([np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi), np.random.uniform(0, 2*np.pi),
               np.random.uniform(0, 10), np.random.uniform(0, 10), np.random.uniform(0, 10)])
        

    def extend(self, qrand, n_iterations):

        '''
        Extend the tree in the direction of qrand, our randomly-sampled node.
            Arguments:
                qrand : Node, randomly sampled node
                n_iterations : number of iterations to simulate our robot moving in, resetting each time.
            Returns:
                qnew : Node, node in state space that moves towards qrand
        '''

        # Find qnear
        print("finding closest existing node to qrand (qnear)...")
        # distance, index = self.kdtree.query(qrand)
        # qnear_state = self.kdtree.data[index] # this is qnear
        # print(f"qnear: {qnear_state}")

        closest_node = None
        min_distance = float('inf')

        for node in self.tree:
            dist = self.distance(node.state, qrand)
            if dist < min_distance:
                closest_node = node
                min_distance = dist

        qnear = closest_node
        qnear_state = closest_node.state
        parent = qnear
        # for node in self.tree:
        #     # print(f"qnear state: {np.array(qnear_state)}, node.state: {np.array(node.state)}\n subtracting: {np.array(qnear_state) - np.array(node.state)}")
        #     if sum(np.array(qnear_state) - np.array(node.state)) < 0.1:
        #         parent = node

        # bridge from that node
        print("simulating to find possible branch to qnew ...")

        valid_sims = {} # initialze simulation dictionary
        # parent_pose = self.robot.joint_angles # save parent pose

        # set the robot (or make a new robot) to have the parent node's positions and velocities.
        IC_robot = Robot(self.robot.link_lengths, qnear_state[:3], qnear_state[3:], self.robot.link_masses) # new robot starting from initial conditions
        IC_holding_torque = IC_robot.calc_holding_torque()

        for i in range(n_iterations):

            # generate random torque values for each joint
            # random_torques = [np.random.randint(-100,100), np.random.randint(-60,60), np.random.randint(-30,30)] # critical value here, min and max torque
            torque_lim0 = 500
            torque_lim1 = 500
            torque_lim2 = 500
            random_torques = [IC_holding_torque[0] + np.random.randint(-torque_lim0,torque_lim0)/10, 
                              IC_holding_torque[1] + np.random.randint(-torque_lim1,torque_lim1)/10, 
                              IC_holding_torque[2] + np.random.randint(-torque_lim2,torque_lim2)/10] # critical value here, min and max torque
            # random_torques = [np.random.randint(-torque_lim0,torque_lim0)/10, 
            #                  np.random.randint(-torque_lim1,torque_lim1)/10, 
            #                  np.random.randint(-torque_lim2,torque_lim2)/10]
            robot_copy = copy.deepcopy(IC_robot)  # create a complete copy of our robot to simulate fresh # TODO if we copy robot with simulate we don't have to do this
            
            timestep = 40 # duration in ms
            steps = 5
            is_valid, robot_states, end_state = self.simulate(robot_copy, random_torques, steps, timestep)

            if is_valid:
                # print(f"endstate: {end_state}")
                dis = self.distance(np.array(qrand), end_state)
                valid_sims[i] = {"end_state": end_state,
                                 "robot_states" : robot_states,
                                 "torques": random_torques,
                                 "distance": dis,
                                 "robot": robot_copy}
        
        print(" Done!")          

        if len(valid_sims) != 0:
            # get lowest distance from valid simulations, thats your new node
            min_distance_key = min(valid_sims, key=lambda x: valid_sims[x]['distance'])

            # add a new node!
            # print(f"qnew state: {valid_sims[min_distance_key]['end_state']}")
            qnew = self.add_node(parent, 
                                 valid_sims[min_distance_key]['end_state'], 
                                 valid_sims[min_distance_key]['torques'], 
                                 valid_sims[min_distance_key]['robot_states'])

            # recalculate kd tree with updated tree
            # self.kdtree = KDTree(np.vstack([node.state for node in self.tree])) 
            # print(f"kdtree data: {self.kdtree.data}")
            for pose in valid_sims[min_distance_key]['robot_states'][3:]:
                robot_copy.visualize(pose, self.map.get_obstacles()[0], timestep)

            print(f"qnew: {qnew}")
            return qnew
        else:
            print("no valid sims")
        

    def check(self, node):

        '''
        Checks to see if our planner has made it close to the goal. (success condition)
            Arguments:
                node : a tree node to compare against the goal node
            Returns:
                sets found flag true if specified node is within the defined threshold distance of the goal
        '''

        goal_threshold = 0.05
        distance = self.distance(node.state, self.goal.state)
        print(f"new node is {round(distance, 3)} units away... ")
        if distance < goal_threshold:
            self.found = True
            self.path = self.construct_path(node)
            print(self.path)


    # === Helper Functions ===
    

    def distance(self, P, Q, metric = "manhattan", bias = .1):

        ''' 
        Distance Metric from points P and Q in our state space
            Arguments:
                P, Q : state vectors of nodes within our tree
                metric : str specifing a certain distance metric (available: manhattan, euclidean)
                bias : balance between angle and angular velocity. Higher value biases angle over angular vel
            Returns:
                float, distance between the two nodes' states
        ''' 

        match metric:
            case "fk":
                pass
            case "manhattan":
                # angle_dif = sum(abs(p - q) for p, q in zip(P[:2], Q[:2])) # whatttt
                angle_dif = sum(abs(p - q) for p, q in zip(P[:3], Q[:3]))

                angle_rate_dif = sum(abs(p - q) for p, q in zip(P[3:], Q[3:]))

                # difference = bias * angle_dif + (1 - bias) * angle_rate_dif
                difference = bias * angle_dif + (1-bias) * angle_rate_dif
                return difference
            
            case "euclidean":
                return np.linalg.norm(P - Q)
            
            case _:
                raise TypeError("Unknown distance metric. Check your spelling? dummy")
    

    def simulate(self, robot, torque, steps, timestep):
        
        '''
        Simulate robot forward in time.
            Arguments:
                robot : robot object, as defined in Robot_Util.py
                torque : 1 x 3 list of joint torques (nm)
                steps : int, number of iterations for the numerical solver
                timestep: number of miliseconds to run each simulaton step
            Returns:
                is_valid, bool, is simulation in collision or not
                robot_states : n x 6 list of joint positions 
                end_state : 1 x 6 np.array of the ending joint positions and velocities
        '''
    
        parent_pose = robot.joint_angles

        robot_states = []
        
        print(f" {(timestep)/1000} s . . .", end='') # print for every sim

        obstacles, obstacle_edges = self.map.get_obstacles()

        # This for loop is purely for visualization, it will only store states every x miliseconds.
        # fwd_dyn() takes torques and a duration, and splits that duration into finer intermediate steps, 
        # returning the end state (angles and velocities). 
        for _ in range(steps):
            # Run one simulation step
            joint_angles, joint_velocities = robot.fwd_dyn(torque, timestep)
   
            # Save intermediate pose
            
            # robot_states.append(np.concatenate((joint_angles.reshape((1,3)), joint_velocities.reshape((1,3))), axis=0).flatten())
            robot_states.append(np.concatenate((joint_angles, joint_velocities), axis=0).T.flatten())
            
            # robot.visualize(joint_angles, self.map.get_obstacles()[0], timestep)
        end_state = robot_states[-1]

        # is_valid_state() does its *own linear interpolation*, between the robot objects current state and a specified parent pose
        is_valid = robot.is_valid_state(parent_pose, obstacles, obstacle_edges)
        if not is_valid:
            print("Ruh roh raggy, simulation collision!")

        return is_valid, robot_states, end_state
    

    def add_node(self, parent, qnew_state, qnew_torques, plotting_poses):

        '''
        Add a valid state to the tree.
            Arguments:
                qnew_state : 1x6 list, state of new node
                qnew_torques : 1x3 list, torques applied to arrive at qnew_state
        '''
        
        # apparently we can't add to kd tree so I guess whenever we query we'll rebuild
        # add node
        new_node = Node(qnew_state, plotting_poses, parent, qnew_torques)
        print(f"ADDED NEW NODE! @ {qnew_state}")
        self.tree.append(new_node)


    def construct_path(self, final_node):
            
        '''
        Compute path cost starting from a start node to an end node
            Arguments:
                final_node - path end node

            Returns:
                path - list of nodes from start to goal
        '''
        
        curr_node = final_node
        path = []

        # Keep tracing back and adding parent nodes to construct path
        while curr_node != None:
            path.append(curr_node)
            parent = curr_node.parent
            curr_node = parent

        path.reverse()
        return path

