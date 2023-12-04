import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# constants
m1 = m2 = 1; l1 = l2 = 1; 
tau1 = tau2 = 0

THETA_1 = THETA_2 = np.pi/3 # 60 degs
THETAD_1 = 2                # 1 rad/s initial angular vel
THETAD_2 = 0

def get_thetadd1(theta1, theta2, thetad1, thetad2, m1, m2, l1, l2, tau1, tau2):
    return l1**2*l2*m2*thetad1**2*np.sin(theta2)*np.cos(theta2)/(l1**2*l2*m1 - l1**2*l2*m2*np.cos(theta2)**2 + l1**2*l2*m2) + l1*l2**2*m2*thetad1**2*np.sin(theta2)/(l1**2*l2*m1 - l1**2*l2*m2*np.cos(theta2)**2 + l1**2*l2*m2) + 2*l1*l2**2*m2*thetad1*thetad2*np.sin(theta2)/(l1**2*l2*m1 - l1**2*l2*m2*np.cos(theta2)**2 + l1**2*l2*m2) + l1*l2**2*m2*thetad2**2*np.sin(theta2)/(l1**2*l2*m1 - l1**2*l2*m2*np.cos(theta2)**2 + l1**2*l2*m2) - l1*tau2*np.cos(theta2)/(l1**2*l2*m1 - l1**2*l2*m2*np.cos(theta2)**2 + l1**2*l2*m2) + l2*tau1/(l1**2*l2*m1 - l1**2*l2*m2*np.cos(theta2)**2 + l1**2*l2*m2) - l2*tau2/(l1**2*l2*m1 - l1**2*l2*m2*np.cos(theta2)**2 + l1**2*l2*m2)

def get_thetadd2(theta1, theta2, thetad1, thetad2, m1, m2, l1, l2, tau1, tau2):
    return -l1**3*l2*m1*m2*thetad1**2*np.sin(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - l1**3*l2*m2**2*thetad1**2*np.sin(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - 2*l1**2*l2**2*m2**2*thetad1**2*np.sin(theta2)*np.cos(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - 2*l1**2*l2**2*m2**2*thetad1*thetad2*np.sin(theta2)*np.cos(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - l1**2*l2**2*m2**2*thetad2**2*np.sin(theta2)*np.cos(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) + l1**2*m1*tau2/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) + l1**2*m2*tau2/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - l1*l2**3*m2**2*thetad1**2*np.sin(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - 2*l1*l2**3*m2**2*thetad1*thetad2*np.sin(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - l1*l2**3*m2**2*thetad2**2*np.sin(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - l1*l2*m2*tau1*np.cos(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) + 2*l1*l2*m2*tau2*np.cos(theta2)/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) - l2**2*m2*tau1/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2) + l2**2*m2*tau2/(l1**2*l2**2*m1*m2 - l1**2*l2**2*m2**2*np.cos(theta2)**2 + l1**2*l2**2*m2**2)

def fwd_dynamics(t):
    tau1 = tau2 = 0 # constant torque input

    # initalize changing values
    theta1 = THETA_1
    theta2 = THETA_2
    thetad1 = THETAD_1
    thetad2 = THETAD_2

    delta_t = 0.1

    for time in np.arange(0, t, delta_t):
        thetadd1 = get_thetadd1(theta1, theta2, thetad1, thetad2, m1, m2, l1, l2, tau1, tau2)
        thetadd2 = get_thetadd2(theta1, theta2, thetad1, thetad2, m1, m2, l1, l2, tau1, tau2)

        theta1 += thetad1 * delta_t
        theta2 += thetad2 * delta_t
        thetad1 += thetadd1 * delta_t 
        thetad2 += thetadd2 * delta_t

    return theta1, theta2

t_start = 0  # Starting time
t_end = 10   # Ending time
step = 0.1   # Time step size
time_values = []  # List to store time values
theta1_values = []  # List to store theta1 values
theta2_values = []  # List to store theta2 values

for t in range(int(t_start * 10), int(t_end * 10) + 1):
    t /= 10.0  # Convert to float
    theta1, theta2 = fwd_dynamics(t)
    time_values.append(t)
    theta1_values.append(theta1)
    theta2_values.append(theta2)

# Plotting theta1 and theta2 over time
# plt.figure(figsize=(10, 6))
# plt.plot(time_values, theta1_values, label='Theta 1')
# plt.plot(time_values, theta2_values, label='Theta 2')
# plt.xlabel('Time')
# plt.ylabel('Joint Angles')
# plt.title('Joint Angles over Time')
# plt.legend()
# plt.grid(True)
# plt.show()

# ANIMATE GRAPH
# fig, ax = plt.subplots()
# line_theta1, = ax.plot([], [], 'r-', label='Theta 1')
# line_theta2, = ax.plot([], [], 'b-', label='Theta 2')
# ax.set_xlim(0, len(theta1_values))  # Adjust the x-axis limits according to your data
# ax.set_ylim(min(min(theta1_values), min(theta2_values)),
#             max(max(theta1_values), max(theta2_values)))  # Adjust the y-axis limits

# # Update function for the animation
# def update(frame):
#     line_theta1.set_data(range(frame), theta1_values[:frame])  # Update Theta 1 values
#     line_theta2.set_data(range(frame), theta2_values[:frame])  # Update Theta 2 values
#     return line_theta1, line_theta2

# # Create the animation
# ani = FuncAnimation(fig, update, frames=len(theta1_values), interval=50, blit=True)

# # Set labels and title
# ax.set_xlabel('Time (s*10)')
# ax.set_ylabel('Joint Angles (rad)')
# ax.set_title('Joint Angle over Time')
# ax.legend()
# plt.grid(True)

# # Show the animation
# plt.show()

# Define the link lengths of the robot arm
# l1 = 1  # Length of link 1
# l2 = 1  # Length of link 2

# # Create a figure and axis for the animation
# fig, ax = plt.subplots()
# line, = ax.plot([], [], 'o-', lw=2)  # Line for the robot arm

# # Set axis limits based on the workspace dimensions
# ax.set_xlim(-3, 3)  # Adjust according to your workspace
# ax.set_ylim(-3, 3)  # Adjust according to your workspace

# # Initialize the robot arm segments
# segment1, = ax.plot([], [], 'r-', lw=2)
# segment2, = ax.plot([], [], 'b-', lw=2)

# # Function to update the robot's arm based on joint angles
# def update(frame):
#     theta1 = theta1_values[frame]  # Get theta1 for the current frame
#     theta2 = theta2_values[frame]  # Get theta2 for the current frame
    
#     # Calculate the positions of joint 1 and joint 2
#     joint1_x = 0
#     joint1_y = 0
#     joint2_x = l1 * np.cos(theta1)
#     joint2_y = l1 * np.sin(theta1)
#     joint3_x = joint2_x + l2 * np.cos(theta1 + theta2)
#     joint3_y = joint2_y + l2 * np.sin(theta1 + theta2)

#     # Update the robot arm segments
#     segment1.set_data([joint1_x, joint2_x], [joint1_y, joint2_y])
#     segment2.set_data([joint2_x, joint3_x], [joint2_y, joint3_y])

#     return segment1, segment2

# # Create the animation
# ani = FuncAnimation(fig, update, frames=len(theta1_values), interval=50, blit=True)

# # Set labels and title
# ax.set_xlabel('X-axis')
# ax.set_ylabel('Y-axis')
# ax.set_title('2R Manipulator Workspace Animation')

# # Show the animation
# plt.show()

# BOTH ANIMATIONS
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Assuming you have already generated theta1_values and theta2_values as lists

# Define the link lengths of the robot arm
l1 = 1  # Length of link 1
l2 = 1  # Length of link 2

# Create a figure and subplots for joint angles and workspace visualization
fig, (ax_joint, ax_workspace) = plt.subplots(1, 2, figsize=(10, 5))

# Set up the workspace plot
ax_workspace.set_xlim(-3, 3)  # Adjust according to your workspace
ax_workspace.set_ylim(-3, 3)  # Adjust according to your workspace
ax_workspace.set_xlabel('X-axis')
ax_workspace.set_ylabel('Y-axis')
ax_workspace.set_title('2R Manipulator Workspace')

# Set up the joint angles plot
ax_joint.set_xlim(0, 100)
ax_joint.set_ylim(-10, 40)
ax_joint.set_xlabel('Time delta (s*10)')
ax_joint.set_ylabel('Joint Angles (rad)')
ax_joint.set_title('Joint Angles over Time')

# Initialize the robot arm segments in the workspace plot
segment1_ws, = ax_workspace.plot([], [], 'r-', lw=2)
segment2_ws, = ax_workspace.plot([], [], 'b-', lw=2)

# Initialize the joint angles plot
line_theta1, = ax_joint.plot([], [], 'r-', label='Theta 1')
line_theta2, = ax_joint.plot([], [], 'b-', label='Theta 2')

# Function to update the animation frames
def update(frame):
    theta1 = theta1_values[frame]  # Get theta1 for the current frame
    theta2 = theta2_values[frame]  # Get theta2 for the current frame
    
    # Calculate the positions of joint 1 and joint 2
    joint1_x = 0
    joint1_y = 0
    joint2_x = l1 * np.cos(theta1)
    joint2_y = l1 * np.sin(theta1)
    joint3_x = joint2_x + l2 * np.cos(theta1 + theta2)
    joint3_y = joint2_y + l2 * np.sin(theta1 + theta2)

    # Update the robot arm segments in the workspace plot
    segment1_ws.set_data([joint1_x, joint2_x], [joint1_y, joint2_y])
    segment2_ws.set_data([joint2_x, joint3_x], [joint2_y, joint3_y])

    # Update the joint angles plot
    line_theta1.set_data(range(frame + 1), theta1_values[:frame + 1])
    line_theta2.set_data(range(frame + 1), theta2_values[:frame + 1])

    return segment1_ws, segment2_ws, line_theta1, line_theta2

# Create the animation
ani = FuncAnimation(fig, update, frames=len(theta1_values), interval=50, blit=True)

# Show the animation
plt.tight_layout()
plt.show()
