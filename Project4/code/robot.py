import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as manimation

from utils import endpoints_to_edges, angle_diff, interpolate_angle, wrap_to_pi
from utils import is_in_polygon, is_intersecting, rotate_about_origin


class Robot:
    """A parent class for all robots"""

    def __init__(self, limits):
        """Initialize by providing limits of each degree of freedom"""
        # Limits in each dof, each limit is defined as (lower, upper, name)
        self.limits = limits
        self.dof = len(limits)

    def forward_kinematics(self, config):
        """Compute the endpoints of the robot given a configuration
        The last endpoint would be used for visualization of the sampling
        """
        raise NotImplementedError

    def get_edges(self):
        """Return the edges of the robot for collision checking"""
        raise NotImplementedError

    def distance(self, config1, config2):
        """Compute the distance between two configurations"""
        raise NotImplementedError

    def interpolate(self, config1, config2, num):
        """Interpolate between two configurations"""
        raise NotImplementedError

    def check_collision(
        self, config1, config2, map_corners, obstacles, obstacle_edges
    ):
        """Check colliding with obstacles between two configurations
        First perform an interpolation between the two configurations,
        then check if any of the interpolated configurations hit obstacles.
       
        arguments:
            config1 - configuration 1
            config2 - configuration 2
            map_corners - corners of the map
            obstacles - list of obstacles
            obstacle_edges - list of edges of obstacles, including map edges
        
        return:
            True if colliding with obstacles between the two configurations
        """
        # Get intepolated configurations in between config1 and config2
        configs_between = self.interpolate(config1, config2)

        # check if any of these configurations hit obstacles
        for config in configs_between:
            if self.check_collision_config(
                config, map_corners, obstacles, obstacle_edges
            ):
                return True
        return False

    def check_collision_config(
        self, config, map_corners, obstacles, obstacle_edges
    ):
        """Check if a configuration is colliding with obstacles. Ensure that all  
        cases are checked. Even ones that might not be present in the given map. 
        arguments:
            config - configuration of the robot
            map_corners - corners of the map
            obstacles - list of obstacles
            obstacle_edges - list of edges of obstacles, including map edges
        
        return:
            True if colliding with obstacles
        """
        # Get the edges of the robot for collision checking
        robot_endpoints = self.forward_kinematics(config)
        robot_edges = self.get_edges(config)

        # Check if any robot endpoint is outside the map
        for endpoint in robot_endpoints:
            if not is_in_polygon(endpoint, map_corners):
                return True

        # Check if the robot endpoint is inside any obstacle
        for obstacle in obstacles:
            for endpoint in robot_endpoints:
                if is_in_polygon(endpoint, obstacle):
                    return True

        # Check if robot edges intersect any obstacle edges
        if is_intersecting(robot_edges, obstacle_edges):
            return True

        return False


    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        """Draw the robot given a configuration on a matplotlib axis.
        This is for visualization purpose only.
        """
        raise NotImplementedError


class PointRobot(Robot):
    """2D Point robot class"""

    def __init__(self):
        """Initialize the robot with no limits in x, y (0, 1000))"""
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y")
        ])

    def forward_kinematics(self, config):
        """Simply return the configuration as the endpoint"""
        return [config]

    def get_edges(self, config):
        """Simply return an empty list"""
        return []

    def distance(self, config1, config2):
        """Euclidean distance"""
        x_diff = config1[0] - config2[0]
        y_diff = config1[1] - config2[1]
        return np.sqrt(x_diff**2 + y_diff**2)

    def interpolate(self, config1, config2, num=5):
        """Interpolate between two configurations"""
        configs_between = zip(
            np.linspace(config1[0], config2[0], num),
            np.linspace(config1[1], config2[1], num)
        )
        return configs_between

    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        ax.scatter(config[0], config[1], s=20, c=edgecolor)


class OmnidirectionalRobot(Robot):
    """Omnidirectional navigation robot class
    Its shape is defined as a rectangle with a width and a height.
    The robot could move in any direction with any angle in a 2D plane.
    """

    def __init__(self, width, height):
        """Initialize the robot with a width and height."""
        self.width = width
        self.height = height
        # Limits in each dof: (x, y, theta)
        # x, y have no limits unless bounded by the map (1000 as default)
        # theta range is (-pi, pi)
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y"),
            (-np.pi, np.pi, "r")
        ])

    def forward_kinematics(self, config):
        """Compute the 4 corner coordinates of the robot given a configuration
        Also attach the center of the robot as the last endpoint.
        The last endpoint would be used for visualization of the sampling.
        arguments:
            config: [x, y, theta] of the rectangle

        return:
            endpoints: 4 corner coordinates of the rectangle and its center
                       [corner1, corner2, corner3, corner4, center]
        """
        # Check configuration shape
        assert len(config) == 3, "Configuration should be (x, y, theta)"

        x, y, theta = config
        endpoints = np.zeros((5, 2))

        # Add center point to end of list of endpoints
        center = [x,y]
        endpoints[4] = center

        # Compute rectangle corners at origin
        corners_origin = [[self.width/2, self.height/2], [-self.width/2, self.height/2], [-self.width/2, -self.height/2], [self.width/2, -self.height/2]]

        # Add corners to list of endpoints
        for n in range(4):
            # rotate corners about origin by theta
            (newX, newY) = rotate_about_origin(corners_origin[n][0], corners_origin[n][1], theta)
            # translate corners by (x,y)
            corner = [x + newX, y + newY]
            # add to list
            endpoints[n] = corner

        # Return list of endpoints
        return endpoints

    def get_edges(self, config):
        """Compute the edges of the robot given a configuration"""

        # Get the 4 corner coordinates
        corners = self.forward_kinematics(config)[:4]

        # Return list of edges
        return endpoints_to_edges(corners, True)

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]

        return:
            distance in R^2 x S^1 space
        """

        x_diff = config1[0] - config2[0]
        y_diff = config1[1] - config2[1]
        theta_diff = wrap_to_pi(config1[2] - config2[2])
        return np.sqrt(x_diff**2 + y_diff**2 + theta_diff**2)
    
    def interpolate(self, config1, config2, num=5):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]
        return:
            list with num number of configs from linear interploation in R^2 x S^1 space
        """

        configs_between = zip(
            np.linspace(config1[0], config2[0], num),
            np.linspace(config1[1], config2[1], num),
            interpolate_angle(config1[2], config2[2], num)
        )
        return configs_between

    def draw_robot(self, ax, config, edgecolor="b", facecolor="pink"):
        # compute corners and draw rectangle
        corners = self.forward_kinematics(config)[:4]
        polygon = Polygon(
            corners, closed=True, edgecolor=edgecolor, facecolor=facecolor
        )
        ax.add_patch(polygon)


class KinematicChain(Robot):
    """Kinematic chain robot class
    A planar robot with a fixed base and pure revolute joints.
    Each link is a line segment.
    """

    def __init__(self, link_lengths, base=[0.1, 0.1]):
        """Initialize with a list of link lengths, and a fixed base."""
        self.base = base
        self.link_lengths = link_lengths
        self.num_joints = len(link_lengths)
        # Limits in each dof
        # assume all to be (-pi, pi)
        super().__init__(limits=[
            (-np.pi, np.pi, "r") for _ in range(self.num_joints)
        ])

    def forward_kinematics(self, config):
        """Compute the joint coordinates given a configuration of joint angles.
        The last endpoint would be used for visualization of the sampling
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of joint coordinates.
        """
        # Initialize the starting point as the fixed base
        joint_positions = [self.base]

        # Add all joint positions to list
        for n in range(self.num_joints):
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
        """Compute the link line segments of the robot given a configuration.
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of line segments representing the link line segments.
        """
        # Check configuration length
        assert (
            len(config) == self.num_joints
        ), "Configuration should match the number of joints"

        # Get the joint positions
        joint_positions = self.forward_kinematics(config)

        # Return list of edges
        return endpoints_to_edges(joint_positions)

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            A Euclidean distance in S^1 x S^1 x ... x S^1 space
        """
        sum = 0
        for n in range(self.num_joints):
            sum += wrap_to_pi(config1[n] - config2[n])**2
        return np.sqrt(sum)

    def interpolate(self, config1, config2, num=10):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            list with num number of configs from linear interploation in S^1 x S^1 x ... x S^1 space.
        """

        joint_interpolations = []

        for n in range(self.num_joints):
            joint_interpolations.append(interpolate_angle(config1[n], config2[n], num))

        configs_between = []

        for j in range(num):
            config = []
            for k in range(self.num_joints):
                config.append(joint_interpolations[k][j])
            configs_between.append(config)
        
        return configs_between
    

    def draw_robot(self, ax, config, edgecolor="b", facecolor="black"):
        # compute joint positions and draw lines
        positions = self.forward_kinematics(config)
        # Draw lines between each joint
        for i in range(len(positions) - 1):
            line = np.array([positions[i], positions[i + 1]])
            ax.plot(line[:, 0], line[:, 1], color=edgecolor)
        # Draw joint
        for i in range(len(positions)):
            ax.scatter(positions[i][0], positions[i][1], s=5, c=facecolor)
