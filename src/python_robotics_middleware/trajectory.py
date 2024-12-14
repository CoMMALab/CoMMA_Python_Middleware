from python_robotics_middleware.waypoint import *
from scipy.interpolate import interp1d
import numpy as np

class Trajectory:
    """
    Represents a trajectory composed of waypoints with interpolation capabilities.
    
    Attributes:
        waypoints (list): A list of Waypoint objects representing the trajectory.
    """
    def __init__(self, waypoints=None):
        """
        Initialize a Trajectory with an optional list of waypoints.
        
        Args:
            waypoints (list, optional): Initial list of Waypoint objects. Defaults to None.
        """
        self.waypoints = waypoints or []
    
    def add_waypoint(self, waypoint):
        """
        Add a waypoint to the trajectory.
        
        Args:
            waypoint (Waypoint): Waypoint to be added to the trajectory.
        """
        self.waypoints.append(waypoint)
    
    def linear_interpolation(self, num_interpolation_points=5):
        """
        Interpolate between existing waypoints and add interpolated points to the collection.
        
        Args:
            num_interpolation_points (int, optional): Number of interpolation points between each pair of waypoints. 
                                                      Defaults to 5.
        """
        # Create a list to store interpolated waypoints
        interpolated_points = []
        
        # Interpolate between each pair of consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            start_waypoint = self.waypoints[i]
            end_waypoint = self.waypoints[i + 1]
            interpolated_points.append(start_waypoint)
            
            # Linearly interpolate positions
            for j in range(1, num_interpolation_points + 1):
                # Linear interpolation
                t = j / (num_interpolation_points + 1)
                interpolated_position = [
                    start_waypoint.position[k] + t * (end_waypoint.position[k] - start_waypoint.position[k])
                    for k in range(3)
                ]
                
                # Create an interpolated waypoint (red color to distinguish)
                interpolated_waypoint = Waypoint(
                    name=f"Interpolated_{start_waypoint.name}_to_{end_waypoint.name}_{j}",
                    position=interpolated_position,
                    color=[1, 0, 0, 0.5],  # Red with some transparency
                    radius=0.03  # Slightly smaller radius
                )
                
                interpolated_points.append(interpolated_waypoint)

        interpolated_points.append(self.waypoints[len(self.waypoints)-1])

        # Replace the original waypoints list
        self.waypoints = interpolated_points

    def spline_interpolation(self, num_interpolation_points=5, spline_degree=3):
        """
        Interpolate between existing waypoints using polynomial spline and add interpolated points to the collection.

        Args:
            num_interpolation_points (int, optional): Number of interpolation points between each pair of waypoints. 
                                                    Defaults to 5.
            spline_degree (int, optional): Degree of the polynomial spline for interpolation. Defaults to 3 (cubic spline).
        """
        # Create a list to store interpolated waypoints
        interpolated_points = []

        # Extract all waypoint positions
        positions = np.array([wp.position for wp in self.waypoints])
        t_values = np.linspace(0, len(self.waypoints) - 1, len(self.waypoints))


        # TODO: potentially rewrite to optimize SPLINE with GPU acceleration
        # Interpolate positions with polynomial spline
        spline_functions = [interp1d(t_values, positions[:, dim], kind=spline_degree) for dim in range(3)]

        # Interpolate between each pair of consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            start_waypoint = self.waypoints[i]
            end_waypoint = self.waypoints[i + 1]
            interpolated_points.append(start_waypoint)

            # Generate t values for interpolation between the two waypoints
            t_start = t_values[i]
            t_end = t_values[i + 1]
            t_interpolated = np.linspace(t_start, t_end, num_interpolation_points + 2)[1:-1]

            for t in t_interpolated:
                interpolated_position = [spline_func(t) for spline_func in spline_functions]

                # Create an interpolated waypoint (red color to distinguish)
                interpolated_waypoint = Waypoint(
                    name=f"Interpolated_{start_waypoint.name}_to_{end_waypoint.name}_{t:.2f}",
                    position=interpolated_position,
                    color=[1, 0, 0, 0.5],  # Red with some transparency
                    radius=0.03  # Slightly smaller radius
                )

                interpolated_points.append(interpolated_waypoint)

        interpolated_points.append(self.waypoints[-1])

        # Replace the original waypoints list
        self.waypoints = interpolated_points


    def _is_between(self, point, start, end):
        """
        Check if a point is between start and end points.
        
        Args:
            point (list): [x,y,z] of the point to check
            start (list): [x,y,z] of the start point
            end (list): [x,y,z] of the end point
        
        Returns:
            bool: True if point is between start and end, False otherwise
        """
        # Use dot product to check if point is between start and end
        for i in range(3):
            if not (min(start[i], end[i]) <= point[i] <= max(start[i], end[i])):
                return False
        return True
    
    def generate_trajectory_visualization(self):
        """
        Generate visual representations of all waypoints in the trajectory.
        
        Returns:
            list: A list of PyBullet waypoint visual IDs.
        """
        return [waypoint.generate_pybullet_waypoint() for waypoint in self.waypoints]
    
    def get_waypoint_positions(self):
        """
        Get a list of all waypoint positions.
        
        Returns:
            list: A list of [x, y, z] positions.
        """
        return [waypoint.position for waypoint in self.waypoints]