import numpy as np
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
ax.set_aspect('equal')


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
    

    def fwd_dyn(self, input_vector, time_steps):
        time_step = 0.001
        joint_torques = np.zeros([self.num_links,1])
        for i in range(self.num_links):
            joint_torques[i] = input_vector[i]
        for x in range(time_steps):
            self.calc_joint_accel(joint_torques)
            self.joint_angles = self.joint_angles + self.joint_velocities * time_step
            self.joint_velocities = self.joint_velocities + self.joint_accelerations * time_step
        return self.joint_angles, self.joint_velocities


    def visualize(self, joint_angles, timestep):

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

        # Plot settings
        ax.set_title('Planar Serial Manipulator')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_xlim(-self.plot_lim, self.plot_lim)
        ax.set_ylim(-self.plot_lim, self.plot_lim)
        plt.grid()
        plt.show(block=False)
        # plt.savefig('manipulator_plot.png')

        plt.pause(timestep * 0.001)