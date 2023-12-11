from Robot_Util import Robot
import numpy as np
from scipy.spatial import KDTree
import copy

# RRT-based kinodynamic planner 

class Node:
    ''' 
    RRT Node object. 
        Attributes:
            state : 1 x 6 list of floats representing theta1, 2, 3, omega 1, 2, 3
            t1, t2, t3, w1, w2, w3 : values for each state (deprecated?)
            parent : Node associated with this Node's parent
            torques_req : 1 x 3 float of torques needed to move to this Node
            plotting_poses : for visualizations: n x 6 array of floats representing slices of our simulation (from the previous Node, to this Node) to be able to plot when we're done
    '''
    def __init__(self, state, plotting_poses, parent=None, torques_req = [0,0,0]):
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
        self.tree = []              # list of nodes as they're added  # TODO: rename 'tree' to RRTtree
        self.start = Node([*robot.joint_angles[:3], *robot.joint_velocities[:3]]) if start.any()==None else Node([*start])  # start pose of robot if given. o/w whatever state the robot is in now.  #TODO: Fix this because it's creating numpy arrays within arrays ;(
        self.goal = Node([*goal])   # desired end pose of robot
        self.found = False          # found flag
        self.path = []              # list of nodes in our found path

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

            self.check(qnew)

            if self.found:
                return self.path
            
        print("Maximum iterations reached. No path found.")
        

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
        parent_pose = self.robot.joint_angles # save parent pose
        
        for i in range(n_iterations):

            # generate random torque values for each joint
            random_torques = np.random.uniform(-10,10,3) # critical value here, min and max torque
            robot_copy = copy.deepcopy(self.robot)  # create a complete copy of our robot to simultate fresh
            
            
            timestep = 50 # duration in ms
            steps = 10 
            is_valid, robot_states, end_state = self.simulate(robot_copy, random_torques, parent_pose, steps, timestep)

            if is_valid:
                dis = self.distance(qrand, np.array(end_state))
                valid_sims[i] = {"end_state": end_state,
                                 "robot_poses" : robot_states,
                                 "torques": random_torques,
                                 "distance": dis}
            
        # get lowest distance from valid simulations, thats your new node
        min_distance_key = min(valid_sims, key=lambda x: valid_sims[x]['distance'])
        print(f"THIS SHOULD BE A STATE: {valid_sims[min_distance_key]['end_state']}")
        self.add_node(parent, valid_sims[min_distance_key]['end state'], valid_sims[min_distance_key]['torques'], valid_sims[min_distance_key]['robot_states'])
        self.kdtree = KDTree(np.vstack([node.state for node in self.tree])) # recalculate kd tree with updated tree
        pass


    def check(self, node):
        '''
        Checks to see if our planner has made it close to the goal. (success condition)
            Returns:

        '''
        goal_threshold = 2
        if self.distance(node, self.goal) < goal_threshold:
            self.found = True
            self.construct_path(node)


    # === Helper Functions ===
    

    def distance(self, P, Q, metric = "manhattan", bias = 0.7):
        ''' 
        Distance Metric from points P and Q in our state space
            Arguments:
                P, Q : Nodes of our tree
                metric : str specifing a certain distance metric 
                bias : balance between angle and angular velocity. Higher value biases angles.
            Returns
                Distance between the two nodes' states
        ''' 
        match metric:
            case "manhattan":
                angle_dif = sum(abs(p - q) for p, q in zip(P[:2], Q[:2]))

                angle_rate_dif = sum(abs(p - q) for p, q in zip(P[3:], Q[3:]))

                difference = bias * angle_dif + (1 - bias) * angle_rate_dif
                
                return difference
            
            case "euclidean":
                return np.linalg.norm(P - Q)
            
            case _:
                raise TypeError("Unknown distance metric. Check your spelling? dummy")
    

    def simulate(self, robot, torque, parent_pose, steps, timestep):
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
                end_state : 1 x 6 list of the ending joint positions and velocities
        '''
    
        robot_states = []
        
        print(f"\tsimulating {(timestep)/1000} s")

        obstacles, obstacle_edges = self.map.get_obstacles()

        # This for loop is purely for visualization, it will only store states every x miliseconds.
        # fwd_dyn() takes torques and a duration, and splits that duration into finer intermediate steps, 
        # returning the end state (angles and velocities). 
        for _ in range(steps):
            # Run one simulation step
            joint_angles, joint_velocities = robot.fwd_dyn(torque, timestep)
            # Save intermediate pose
            robot_states.append(np.concatenate((joint_angles.reshape((1,3)), joint_velocities.reshape((1,3))), axis=0))
        
        end_state = robot_states[-1]

        # isValidState() does its *own linear interpolation*, between the robot objects current state and a 
        # specified parent pose
        is_valid = robot.isValidState(parent_pose, obstacles, obstacle_edges)
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
        self.tree.append(Node(qnew_state, plotting_poses, parent, qnew_torques))


    def construct_path(self, final_node):
        path = []

        for i in range(self.tree):
            point = self.get_new_point(0.09)

            if self.map_array[point[0]][point[1]] == 0:
                continue

            qrand = Node(point[0], point[1])

            qnear = self.get_nearest_node(qrand) # first time around, this is the start

            row = 0
            col = 0

            if qnear.col == qrand.col:
                new_point_row = qrand.row + step
                new_point_col = qrand.col
            else:
                direction_vector = np.array([qrand.row - qnear.row, qrand.col - qnear.col])
                direction_norm = np.linalg.norm(direction_vector)

                unit_direction = direction_vector / direction_norm

                row = int(qnear.row + step * unit_direction[0])
                col = int(qnear.col + step * unit_direction[1])
            
            
            if not self.check_collision(qnear, Node(row, col)):
                qnew = Node(row, col)
                qnew.parent = qnear
                self.vertices.append(qnew)

                if len(self.get_neighbors(self.goal, step)) != 0:
                    if not self.check_collision(qnew, self.goal):
                        self.goal.parent = qnew
                        self.found = True

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        return path
