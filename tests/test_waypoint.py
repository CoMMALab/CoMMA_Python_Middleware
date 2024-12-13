
import pybullet as p
import pybullet_data
import time
import numpy as np
import os 
from python_robotics_middleware import Waypoint, are_tuples_close

def visualize_with_waypoints(data, urdf, waypoints, visualize=True, delay=False):
    """
    Visualize pre-computed joint configurations in PyBullet with waypoints.

    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        waypoints: List of Waypoint objects.
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
        # Use the last linka as the end effector that needs to contact the waypoint 
        end_effector_link_index = num_joints-1

        if end_effector_link_index is None:
            raise ValueError("End effector not found in the robot's URDF.")
        
        # Generate and load waypoints
        waypoint_ids = [waypoint.generate_pybullet_waypoint() for waypoint in waypoints]

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
                    print(f"contact met at waypoint {waypoints[i].name}")
                    waypoints[i].met = True
                    p.removeBody(waypoint_id)  # Remove the waypoint from the simulation
                        
            if all(waypoint.met for waypoint in waypoints):
                break

            if delay:
                time.sleep(0.01)

if __name__ == "__main__":
    search_path = pybullet_data.getDataPath()

    # Define URDF and dataset
    urdf = "../franka/fp3_franka_hand.urdf"
    data = [
        {'joint_angles': [0.0, -0.5, 0.5, -1.0, 0.0, 0.8, 0.0]},  # Example pose 1
        {'joint_angles': [0.2, -0.3, 0.7, -0.9, 0.1, 0.5, 0.0]},  # Example pose 2
        # Add more poses here...
    ]

    # Define waypoints
    waypoints = [
        Waypoint(name="wp1", position=[0.5, 0.5, 0.2]),
        Waypoint(name="wp2", position=[0.6, 0.5, 0.3]),
        Waypoint(name="wp3", position=[0.7, 0.5, 0.4]),
    ]

    visualize_with_waypoints(data=data, urdf=urdf, waypoints=waypoints)
