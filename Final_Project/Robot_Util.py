import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from Utils import interpolate_angle, is_in_polygon, is_intersecting, endpoints_to_edges, rotate_about_origin


# Parameters
fwd_dyn_time_step = 0.001
joint_velocity_limit = 10 # radians/second


# Initialize figure for animation
fig, ax = plt.subplots()
ax.set_aspect('equal')


# Robot class for n-link planar manipulator (n = [1, 3])
class Robot:
    def __init__(self, link_lengths, init_angles, init_velocities, link_masses):

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
        

    # Calculate the mass matrix
    def calc_mass_matrix(self):
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


    # Calculate the Coriolis matrix
    def calc_coriolis_matrix(self):
        if self.num_links == 1:
            self.C[0] = 0
        if self.num_links == 2:
            self.C[0] = -self.link_masses[1]*self.link_lengths[0]*self.link_lengths[1]*np.sin(self.joint_angles[1])*(2*self.joint_velocities[0]*self.joint_velocities[1] + self.joint_velocities[1]**2)
            self.C[1] = self.link_masses[1]*self.link_lengths[0]*self.link_lengths[1]*self.joint_velocities[0]**2*np.sin(self.joint_angles[1])
        if self.num_links == 3:
            self.C[0] = -self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.sin(self.joint_angles[1])*self.joint_velocities[1]**2 - self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.sin(self.joint_angles[1])*self.joint_velocities[1]**2 - self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[2]**2 - self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[1]**2 - self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[2]**2 - 2*self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.sin(self.joint_angles[1])*self.joint_velocities[0]*self.joint_velocities[1] - 2*self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.sin(self.joint_angles[1])*self.joint_velocities[0]*self.joint_velocities[1] - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[2] - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[1]*self.joint_velocities[2] - 2*self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[1] - 2*self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[2] - 2*self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[1]*self.joint_velocities[2]
            self.C[1] = self.link_lengths[0]*self.link_lengths[1]*self.link_masses[1]*np.sin(self.joint_angles[1])*self.joint_velocities[0]**2 + self.link_lengths[0]*self.link_lengths[1]*self.link_masses[2]*np.sin(self.joint_angles[1])*self.joint_velocities[0]**2 - self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[2]**2 + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]**2 - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[2] - 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[1]*self.joint_velocities[2]
            self.C[2] = self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]**2 + self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[1]**2 + self.link_lengths[0]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[1] + self.joint_angles[2])*self.joint_velocities[0]**2 + 2*self.link_lengths[1]*self.link_lengths[2]*self.link_masses[2]*np.sin(self.joint_angles[2])*self.joint_velocities[0]*self.joint_velocities[1]


    # Calculate the gravity matrix
    def calc_gravity_matrix(self):
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


    # Compute joint accelerations from joint positions, joint velocities, and torques
    def calc_joint_accel(self, joint_torques):
        self.calc_mass_matrix()
        self.calc_coriolis_matrix()
        self.calc_gravity_matrix()
        self.joint_accelerations = np.matmul(np.linalg.inv(self.M), joint_torques - self.C - self.G)
    

    # Use forward dynamics to calculate the robot state from an initial state, torque input vector, and time step
    def fwd_dyn(self, input_vector, time_steps):
        joint_torques = np.zeros([self.num_links,1])
        for i in range(self.num_links):
            joint_torques[i] = input_vector[i]
        for x in range(time_steps):
            self.calc_joint_accel(joint_torques)
            self.joint_angles = self.joint_angles + self.joint_velocities * fwd_dyn_time_step
            self.joint_velocities = self.joint_velocities + self.joint_accelerations * fwd_dyn_time_step
        return self.joint_angles, self.joint_velocities
    

    # Calculate positions of ends of each link
    def forward_kinematics(self, config):
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


    # Get list of robot edges
    def get_edges(self, config):

        # Get the joint positions
        joint_positions = self.forward_kinematics(config)

        # Return list of edges
        return endpoints_to_edges(joint_positions)
    

    # Interpolate between two configurations
    def interpolate(self, config1, config2, num=10):

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
        # Get the edges of the robot for collision checking
        robot_endpoints = self.forward_kinematics(config)
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

        # Get intepolated configurations in between config1 and config2
        configs_between = self.interpolate(parent_config, self.joint_angles)

        # check if any of these configurations hit obstacles
        for config in configs_between:
            if self.check_collision_config(config, obstacles, obstacle_edges):
                return True
        return False
    

    # Check if the robot is in a valid configuration (within limits and not in collision with obstacles)
    def isValidState(self, parent_config, obstacles, obstacle_edges):
        ## Check velocity bounds
        for joint_velocity in self.joint_velocities:
            if abs(joint_velocity) > joint_velocity_limit:
                return False
        ## Check collisions
        if self.check_collision(parent_config, obstacles, obstacle_edges):
            return False
        ## If both conditions pass, return true
        return True
    

    # Display an animation of the robot's motion
    def visualize(self, joint_angles, obstacles, timestep):

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

