import pybullet as p
import pybullet_data
import time
import numpy as np
import os
import math 
from python_robotics_middleware import Waypoint
from python_robotics_middleware import Trajectory 

def generate_dynamic_waypoints(num_points=5, base_height=0.5, amplitude=0.2, frequency=1.5):
    """
    Generate a set of dynamic waypoints that follow a more complex path.
    
    Args:
        num_points (int): Number of waypoints to generate.
        base_height (float): Base Z-height of the trajectory.
        amplitude (float): Amplitude of the sinusoidal variation.
        frequency (float): Frequency of the sinusoidal variation.
    
    Returns:
        list: A list of Waypoint objects.
    """
    waypoints = []
    
    for i in range(num_points):
        # Calculate x and y positions in a spiral-like pattern
        x = 0.5 + i * 0.15 * math.cos(i * frequency)
        y = 0.5 + i * 0.15 * math.sin(i * frequency)
        
        # Z-position with sinusoidal variation
        z = base_height + amplitude * math.sin(i * frequency)
        
        # Create waypoint with descriptive name
        waypoint = Waypoint(
            name=f"wp{i+1}", 
            position=[x, y, z],
            color=[0, 1, 0, 0.75]  # Green color with some transparency
        )
        
        waypoints.append(waypoint)
    
    return waypoints


def visualize_trajectory(urdf, trajectory, visualize=True, delay=False):
    """
    Visualize a trajectory with waypoints in PyBullet.
    
    Args:
        urdf: Path to the robot URDF file.
        trajectory: Trajectory object containing waypoints.
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81 / 1000)
        p.setTimeStep(1 / 240.0)
        
        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)
        
        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
        
        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)
        num_joints = p.getNumJoints(robot_id)
        
        # Use the last link as the end effector that needs to contact the waypoint
        end_effector_link_index = num_joints - 1
        
        if end_effector_link_index is None:
            raise ValueError("End effector not found in the robot's URDF.")
        
        # Generate and load waypoints
        waypoint_ids = trajectory.generate_trajectory_visualization()
        
        # Main simulation loop
        while True:
            p.stepSimulation()
            
            # Check for collisions between waypoints and the robot
            for i, waypoint_id in enumerate(waypoint_ids):
                contact_points = p.getContactPoints(
                    bodyA=robot_id,
                    linkIndexA=end_effector_link_index,
                    bodyB=waypoint_id
                )
                
                if contact_points:
                    print(f"Contact met at waypoint {trajectory.waypoints[i].name}")
                    trajectory.waypoints[i].met = True
                    p.removeBody(waypoint_id)
            
            # Break if all waypoints have been met
            if all(waypoint.met for waypoint in trajectory.waypoints):
                break
            
            if delay:
                time.sleep(0.01)


if __name__ == "__main__":
    # Define search path
    search_path = pybullet_data.getDataPath()
    
    # Define URDF
    urdf = "../franka/fp3_franka_hand.urdf"
    
    # Create trajectory
    trajectory = Trajectory()
    
    # Generate dynamic waypoints
    dynamic_waypoints = generate_dynamic_waypoints(num_points=7)
    
     # Add original waypoints
    original_waypoints = [
        Waypoint(name="wp1", position=[0.5, 0.5, 0.2]),
        Waypoint(name="wp2", position=[0.6, 0.5, 0.3]),
        Waypoint(name="wp3", position=[0.7, 0.5, 0.4]),
    ]
    
    # Add waypoints to trajectory
    # for wp in original_waypoints:
    #     trajectory.add_waypoint(wp)

    # Add waypoints to trajectory
    for wp in dynamic_waypoints:
        trajectory.add_waypoint(wp)
    
    # Interpolate waypoints
    # trajectory.linear_interpolation(num_interpolation_points=3)
    trajectory.spline_interpolation(num_interpolation_points=3, spline_degree=5)
    
    # Visualize trajectory
    visualize_trajectory(urdf=urdf, trajectory=trajectory)