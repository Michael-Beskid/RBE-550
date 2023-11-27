import numpy as np
import matplotlib.pyplot as plt


## Forward Dynamics
def skew(w):
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])


class Robot:
    def __init__(self, link_lengths, joint_angles):

        self.link_lengths = link_lengths
        self.num_links = len(link_lengths)

        self.joint_angles = joint_angles
        self.joint_velocities = np.zeros(self.num_links)
        self.joint_accelerations = np.zeros(self.num_links)

        self.M = 0  # Need to construct mass matrix


    def fwd_dyn(self, joint_torques):
        pass


    ## Visalize Manipulator
    def visualize(self):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')

        # Starting point
        points = np.array([[0, 0]])

        # Iterate through each link and joint angle
        for i in range(len(self.link_lengths)):
            angle = np.sum(self.joint_angles[:i + 1])

            # Calculate endpoint of the current link
            x = points[-1, 0] + self.link_lengths[i] * np.cos(angle)
            y = points[-1, 1] + self.link_lengths[i] * np.sin(angle)

            # Store the endpoint
            points = np.vstack([points, [x, y]])

            # Plot the manipulator link
            ax.plot([points[i, 0], points[i + 1, 0]], [points[i, 1], points[i + 1, 1]], 'bo-')

        # Set plot limits to max extension length of manipulator
        plot_lim = 0
        for length in self.link_lengths:
            plot_lim += length

        # Plot settings
        ax.set_title('Planar Serial Manipulator')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_xlim(0,plot_lim)
        ax.set_ylim(0,plot_lim)
        plt.grid()
        plt.show()
        # plt.savefig('manipulator_plot.png')


# def fk(links, angles):
#     num_links = len(links)
#     T = np.identity(4)
#     identity = np.identity(3)

#     for i in range(num_links):
#         theta = angles[i]
#         link_length = links[i]

#         # Define joint twist for revolute joint
#         joint_twist = np.array([0, 0, link_length])

#         # Define joint axis (Z-axis for simplicity)
#         joint_axis = np.array([0, 0, 1])

#         # Construct exponential twist matrix
#         w_skew = skew(joint_twist)
#         R = np.identity(3) + np.sin(theta) * w_skew + (1 - np.cos(theta)) * (w_skew @ w_skew) 
#         p = (identity * theta + (1 - np.cos(theta)) * w_skew + (theta - np.sin(theta)) * (w_skew @ w_skew)) @ joint_axis

#         # Construct transformation matrix
#         Ti = np.identity(4)
#         Ti[:3, :3] = R
#         Ti[:3, 3] = p

#         # Update total transformation matrix
#         T = np.dot(T, Ti)

#     return T