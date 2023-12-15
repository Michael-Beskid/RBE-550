import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from Utils import interpolate_angle, is_in_polygon, is_intersecting, endpoints_to_edges, rotate_about_origin


# Parameters
fwd_dyn_time_step = 0.001 # seconds
joint_velocity_limit = 100 # rad/sec


# Initialize figure for animation
fig, ax = plt.subplots()
ax.set_aspect('equal')


class Robot:

    ''' 
    Robot object used to simulate the dynamics of a particular n-link (n = [1,2,3]) planar manipulator
        Attributes:
            link_lengths : A list of the lengths of each link
            num_links : The number of links the manipulator has
            joint_angles : A list of angualr positions for each joint
            joint_velocities : A list of angular velocities for each joint
            joint_accelerations : A list of angualr accelerations for each joint
            link_masses : A list of the masses of each link
            M : The mass matrix for the current configuration
            C : The Coriolis matrix for the current configuration
            G : The gravity matrix for the current configuration
        Methods:
            init() : Instantiate a new Robot object with specified parameters.
            calc_mass_matrix() : Compute the mass matrix for the current configuration.
            calc_coriolis_matrix() : Compute the Coriolis matrix for the current configuration of the manipulator.
            calc_gravity_matrix() : Compute the gravity matrix for the current configuration of the manipulator.
            fwd_dyn() : Simulate the robot forward in time using the dynamical model and update the robot's state.
            fwd_kin() : Calculate the positions of the ends of each link for an input configuration.
            get_edges() : Produce a list of line segments that make up the manipulator for a specified configuration.
            interpolate() : Generate a list of manipulator configurations interpolated between two specified configurations.
            check_collision_config() : Check if a particualr configuration of the manipulator is in collision with any obstacles.
            check_collision() : Checks if collisions result in moving the manipulator from a parent configration to its current configuration.
            is_valid_state() : Checks if a specified movement of the manipulator is valid. Checks velocity limits and obstacle collisions.
            calc_holding_torque() : Computes the holding torque to be applied at each joint to hold the manipulator in place at its current configuration.
            visualize() : Display a visualization plot of the manipulator for a specified configuration.
    '''
    
    def __init__(self, link_lengths, init_angles, init_velocities, link_masses):

        '''
        Instantiate a new Robot object with specified parameters.
            Arguments:
                link_lengths : a list of lengths for each link in the manipulator
                init_angles : a list of initial angular positions for the joints
                init_velocities : a list of initial angular velocities for the joints
                link_masses : a list of masses for each link in the manipulator
        '''

        self.link_lengths = link_lengths
        self.num_links = len(link_lengths)

        self.joint_angles = np.zeros([self.num_links,1])
        self.joint_velocities = np.zeros([self.num_links,1])
        self.joint_accelerations = np.zeros([self.num_links,1])
        for i in range(self.num_links):
            self.joint_angles[i] = init_angles[i]
            self.joint_velocities[i] = init_velocities[i]

        self.link_masses = link_masses

        self.M = np.zeros([self.num_links, self.num_links]) # Compute mass matrix
        self.calc_mass_matrix()

        self.C = np.zeros([self.num_links, 1]) # Compute Coriolis matrix
        self.calc_coriolis_matrix()

        self.G = np.zeros([self.num_links, 1]) # Compute gravity matrix
        self.calc_gravity_matrix()

        # Set plot limits to max extension length of manipulator
        self.plot_lim = 0
        for length in self.link_lengths:
            self.plot_lim += length
        

    def calc_mass_matrix(self):

        '''
        Compute the mass matrix for the current configuration of the manipulator.
        '''
        
        if self.num_links == 1:
            self.M[0] = self.link_masses[0]*self.link_lengths[0]**2
        if self.num_links == 2:
            self.M[0][0] = self.link_masses[0]*self.link_lengths[0]**2 + self.link_masses[1]*(self.link_lengths[0]**2 + 2*self.link_lengths[0]*self.link_lengths[1]*np.cos(self.joint_angles[1]) + self.link_lengths[1]**2)
            self.M[0][1] = self.link_masses[1]*(self.link_lengths[0]*self.link_lengths[1]*np.cos(self.joint_angles[1]) + self.link_lengths[1]**2)
            self.M[1][0] = self.link_masses[1]*(self.link_lengths[0]*self.link_lengths[1]*np.cos(self.joint_angles[1]) + self.link_lengths[1]**2)
            self.M[1][1] = self.link_masses[1]*self.link_lengths[1]**2
        if self.num_links == 3:
            self.M[0][0] = (self.link_lengths[0]**2*self.link_masses[0] + self.link_lengths[0]**2*self.link_masses[1] + self.link_lengths[0]**2*self.link_masses[2] + self.link_lengths[1]**2*self.link_masses[1] + self.link_lengths[1]**2*self.link_masses[2] + self.link_lengths[2]**2*self.link_masses[2] + 2*self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.cos(self.joint_angles[1]) + 2*self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.cos(self.joint_angles[1]) + 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[2]) + 2*self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[1] + self.joint_angles[2]))
            self.M[0][1] = (self.link_lengths[1]**2*self.link_masses[1] + self.link_lengths[1]**2*self.link_masses[2] + self.link_lengths[2]**2*self.link_masses[2] + self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.cos(self.joint_angles[1]) + self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.cos(self.joint_angles[1]) + 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[2]) + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[1] + self.joint_angles[2]))
            self.M[0][2] = (self.link_lengths[2]**2*self.link_masses[2] + self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[2]) + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[1] + self.joint_angles[2]))
            self.M[1][0] = (self.link_lengths[1]**2*self.link_masses[1] + self.link_lengths[1]**2*self.link_masses[2] + self.link_lengths[2]**2*self.link_masses[2] + self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.cos(self.joint_angles[1]) + self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.cos(self.joint_angles[1]) + 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[2]) + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[1] + self.joint_angles[2]))
            self.M[1][1] = (self.link_lengths[1]**2*self.link_masses[1] + self.link_lengths[1]**2*self.link_masses[2] + self.link_lengths[2]**2*self.link_masses[2] + 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[2]))
            self.M[1][2] = (self.link_masses[2]*self.link_lengths[2]**2 + self.link_lengths[1]*self.link_masses[2]*np.cos(self.joint_angles[2])*self.link_lengths[2])
            self.M[2][0] = (self.link_lengths[2]**2*self.link_masses[2] + self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[2]) + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[1] + self.joint_angles[2]))
            self.M[2][1] = (self.link_masses[2]*self.link_lengths[2]**2 + self.link_lengths[1]*self.link_masses[2]*np.cos(self.joint_angles[2])*self.link_lengths[2])
            self.M[2][2] = self.link_lengths[2]**2*self.link_masses[2]


    def calc_coriolis_matrix(self):

        '''
        Compute the Coriolis matrix for the current configuration of the manipulator.
        '''
        
        if self.num_links == 1:
            self.C[0] = 0
        if self.num_links == 2:
            self.C[0] = -self.link_masses[1]*self.link_lengths[0]*self.link_lengths[1]*np.sin(self.joint_angles[1])*(2*self.joint_velocities[0]*self.joint_velocities[1] + self.joint_velocities[1]**2)
            self.C[1] = self.link_masses[1]*self.link_lengths[0]*self.link_lengths[1]*self.joint_velocities[0]**2*np.sin(self.joint_angles[1])
        if self.num_links == 3:
            self.C[0] = -self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.sin(self.joint_angles[1])*self.joint_velocities[1]**2 - self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.sin(self.joint_angles[1])*self.joint_velocities[1]**2 - self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[2]**2 - self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[1]**2 - self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[2]**2 - 2*self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.sin(self.joint_angles[1])*self.joint_velocities[0]*self.joint_velocities[1] - 2*self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.sin(self.joint_angles[1])*self.joint_velocities[0]*self.joint_velocities[1] - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[2] - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[1]*self.joint_velocities[2] - 2*self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[1] - 2*self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[2] - 2*self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[1]*self.joint_velocities[2]
            self.C[1] = self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.sin(self.joint_angles[1])*self.joint_velocities[0]**2 + self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.sin(self.joint_angles[1])*self.joint_velocities[0]**2 - self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[2]**2 + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]**2 - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[2] - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[1]*self.joint_velocities[2]
            self.C[2] = self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]**2 + self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[1]**2 + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]**2 + 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[1]


    def calc_gravity_matrix(self):

        '''
        Compute the gravity matrix for the current configuration of the manipulator.
        '''
        
        g = 9.81
        if self.num_links == 1:
            self.G[0] = self.link_masses[0]*self.link_lengths[0]*g*np.cos(self.joint_angles[0])
        if self.num_links == 2:
            self.G[0] = (self.link_masses[0] + self.link_masses[1])*self.link_lengths[0]*g*np.cos(self.joint_angles[0]) + self.link_masses[1]*g*self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1])
            self.G[1] = self.link_masses[1]*g*self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1])
        if self.num_links == 3:
            self.G[0] = (self.link_lengths[0]*self.link_masses[0]*np.cos(self.joint_angles[0]) + self.link_lengths[0]*self.link_masses[1]*np.cos(self.joint_angles[0]) + self.link_lengths[0]*self.link_masses[2]*np.cos(self.joint_angles[0]) + self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[0] + self.joint_angles[1] + self.joint_angles[2]) + self.link_lengths[1]*self.link_masses[1]*np.cos(self.joint_angles[0] + self.joint_angles[1]) + self.link_lengths[1]*self.link_masses[2]*np.cos(self.joint_angles[0] + self.joint_angles[1]))*g
            self.G[1] = (self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[0] + self.joint_angles[1] + self.joint_angles[2]) + self.link_lengths[1]*self.link_masses[1]*np.cos(self.joint_angles[0] + self.joint_angles[1]) + self.link_lengths[1]*self.link_masses[2]*np.cos(self.joint_angles[0] + self.joint_angles[1]))*g
            self.G[2] = self.link_lengths[2]*self.link_masses[2]*np.cos(self.joint_angles[0] + self.joint_angles[1] + self.joint_angles[2])*g


    def calc_joint_accel(self, joint_torques):

        '''
        Compute the joint accelerations resulting from an input torque vector.
            Arguments:
                joint_torques : list of torques applied at each joint 
        '''
        
        self.calc_mass_matrix()
        self.calc_coriolis_matrix()
        self.calc_gravity_matrix()
        self.joint_accelerations = np.matmul(np.linalg.inv(self.M), joint_torques - self.C - self.G)
    

    def fwd_dyn(self, input_vector, time_steps):

        '''
        Simulate the robot forward in time using the dynamical model and update the robot's state.
            Arguments:
                input_vector : the initial state to begin the simulation from
                time_steps : the number of finite 0.01 second steps to run the simulation for
            Returns:
                joint_angles : a list of updated angular positions of the joints 
                joint_velocities : a list of updated angular velocities of the joints 
        '''
        
        joint_torques = np.zeros([self.num_links,1])
        for i in range(self.num_links):
            joint_torques[i] = input_vector[i]
        for x in range(time_steps):
            self.calc_joint_accel(joint_torques)
            self.joint_angles = self.joint_angles + self.joint_velocities * fwd_dyn_time_step
            self.joint_velocities = self.joint_velocities + self.joint_accelerations * fwd_dyn_time_step
        return self.joint_angles, self.joint_velocities
    

    def fwd_kin(self, config):

        '''
        Calculate the positions of the ends of each link for an input configuration.
            Arguments:
                config : a list of joint angles specifying the configuration of the manipulator
            Returns:
                joint_positions : a list of [x,y] points located at the ends of each link (ordered 1--> n)
        '''
        
        # Initialize the starting point as the fixed base
        joint_positions = [(0, 0)]

        # Add all joint positions to list
        for n in range(self.num_links):
            # Compute angle
            angle = 0
            for j in range(n+1):
                angle += config[j]
            # Compute link end point with respect to origin
            (newX, newY) = rotate_about_origin(self.link_lengths[n], 0, angle)
            # Translate link from origin to previous link endpoint
            new_joint_position = [joint_positions[n][0] + newX, joint_positions[n][1] + newY]
            # Add to list
            joint_positions.append(new_joint_position)

        return joint_positions


    def get_edges(self, config):

        '''
        Produce a list of line segments that make up the manipulator for a specified configuration.
            Arguments:
                config : a list of joint angles specifying the configuration of the manipulator
            Returns:
                edges: a list of line segments which make up the manipulator in the specified configuration
        '''
        
        # Get the joint positions
        joint_positions = self.fwd_kin(config)

        # Return list of edges
        edges = endpoints_to_edges(joint_positions)
        return edges
    

    def interpolate(self, config1, config2, num=10):

        '''
        Generate a list of manipulator configurations interpolated between two specified configurations.
            Arguments:
                config1 :  a list of joint angles specifying the starting configuration of the manipulator
                config2 :  a list of joint angles specifying the final configuration of the manipulator
                num : the number of interpolated configurations to generate
            Returns:
                configs_between: a list of lists of joint angles specifying the interpolated configurations
        '''
        
        joint_interpolations = []

        for n in range(self.num_links):
            joint_interpolations.append(interpolate_angle(config1[n], config2[n], num))

        configs_between = []

        for j in range(num):
            config = []
            for k in range(self.num_links):
                config.append(joint_interpolations[k][j])
            configs_between.append(config)
        
        return configs_between
    

    def check_collision_config(self, config, obstacles, obstacle_edges):

        '''
        Check if a particualr configuration of the manipulator is in collision with any obstacles.
            Arguments:
                config : a list of joint angles specifying the manipulator configuration to check
                obstacles : a list of polygonal obstacles
                obstacle_edges : a list of the line segments which make up all obstacles
            Returns:
                True if the manipulator is in collision with an obstacle
        '''
        
        # Get the edges of the robot for collision checking
        robot_endpoints = self.fwd_kin(config)
        robot_edges = self.get_edges(config)

        # Check if the robot endpoint is inside any obstacle
        for obstacle in obstacles:
            for endpoint in robot_endpoints:
                if is_in_polygon(endpoint, obstacle):
                    return True

        # Check if robot edges intersect any obstacle edges
        if is_intersecting(robot_edges, obstacle_edges):
            return True

        return False


    def check_collision(self, parent_config, obstacles, obstacle_edges):

        '''
        Checks if collisions result in moving the manipulator from a parent configration to its current configuration.
            Arguments:
                parent_config : a list of joint angles specifying the starting configuration of the manipulator
                obstacles : a list of polygonal obstacles
                obstacle_edges : a list of the line segments which make up all obstacles
            Returns:
                True if the movement results in any collisions with obstacles
        '''

        # Get intepolated configurations in between config1 and config2
        configs_between = self.interpolate(parent_config, self.joint_angles)

        # check if any of these configurations hit obstacles
        for config in configs_between:
            if self.check_collision_config(config, obstacles, obstacle_edges):
                return True
        return False
    

    def is_valid_state(self, parent_config, obstacles, obstacle_edges):

        '''
        Checks if a specified movement of the manipulator is valid. Checks velocity limits and obstacle collisions.
            Arguments:
                parent_config : a list of joint angles specifying the starting configuration of the manipulator
                obstacles : a list of polygonal obstacles
                obstacle_edges : a list of the line segments which make up all obstacles
            Returns:
                True if the movement from the parent state to the current state is valid
        '''
        
        ## Check velocity bounds
        for joint_velocity in self.joint_velocities:
            if abs(joint_velocity) > joint_velocity_limit:
                return False
        ## Check collisions
        if self.check_collision(parent_config, obstacles, obstacle_edges):
            return False
        ## If both conditions pass, return true
        return True
    

    # Compute holding torque from current configuration
    def calc_holding_torque(self):

        '''
        Computes the holding torque to be applied at each joint to hold the manipulator in place at its current configuration.
            Returns:
                holding_torque : a list of torque values to be applied at each joint to hold the current configuration
        '''
        
        g = 9.81

        if self.num_links == 1:
            tau1 = self.link_masses[0]*g*(self.link_lengths[0]*np.cos(self.joint_angles[0]))
            holding_torque = [tau1]
        if self.num_links == 2:
            tau1 = self.link_masses[0]*g*(self.link_lengths[0]*np.cos(self.joint_angles[0])) + self.link_masses[1]*g*(self.link_lengths[0]*np.cos(self.joint_angles[0]) + self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1]))
            tau2 = self.link_masses[1]*g*(self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1]))
            holding_torque = [tau1, tau2]
        if self.num_links == 3:
            tau1 = self.link_masses[0]*g*(self.link_lengths[0]*np.cos(self.joint_angles[0])) + self.link_masses[1]*g*(self.link_lengths[0]*np.cos(self.joint_angles[0]) + self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1])) + self.link_masses[2]*g*(self.link_lengths[0]*np.cos(self.joint_angles[0]) + self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1]) + self.link_lengths[2]*np.cos(self.joint_angles[0] + self.joint_angles[1] + self.joint_angles[2]))
            tau2 = self.link_masses[1]*g*(self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1])) + self.link_masses[2]*g*(self.link_lengths[1]*np.cos(self.joint_angles[0] + self.joint_angles[1]) + self.link_lengths[2]*np.cos(self.joint_angles[0] + self.joint_angles[1] + self.joint_angles[2]))
            tau3 = self.link_masses[2]*g*(self.link_lengths[2]*np.cos(self.joint_angles[0] + self.joint_angles[1] + self.joint_angles[2]))
            holding_torque = [tau1, tau2, tau3]
        
        return holding_torque
    

    def visualize(self, joint_angles, obstacles, timestep):

        '''
        Display a visualization plot of the manipulator for a specified configuration.
            Arguments:
                joint_angles : a list of joint angles specifying the configuration of the manipulator to show
                obstacles : a list of polygonal obstacles to illustrate in the workspace
                timestep : the length of time to display the plot
        '''

        # Clear the plot
        ax.clear()
        
        # Starting point
        points = np.array([[0, 0]])

        # Link plot styles
        styles = ['bo-', 'ro-', 'go-']

        # Iterate through each link and joint angle
        for i in range(len(self.link_lengths)):
            angle = np.sum(joint_angles[:i + 1])

            # Calculate endpoint of the current link
            x = points[-1, 0] + self.link_lengths[i] * np.cos(angle)
            y = points[-1, 1] + self.link_lengths[i] * np.sin(angle)

            # Store the endpoint
            points = np.vstack([points, [x, y]])

            # Plot the manipulator link
            ax.plot([points[i, 0], points[i + 1, 0]], [points[i, 1], points[i + 1, 1]], styles[i])

        # Draw each obstacle as a polygon
        for obstacle in obstacles:
            polygon = Polygon(
                obstacle, closed=True, edgecolor='black', facecolor='gray'
            )
            ax.add_patch(polygon)

        # Plot settings
        ax.set_title('Planar Serial Manipulator')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_xlim(-self.plot_lim, self.plot_lim)
        ax.set_ylim(-self.plot_lim, self.plot_lim)
        # plt.grid()
        plt.show(block=False)
        # plt.savefig('manipulator_plot.png')

        plt.pause(timestep * 0.001)

